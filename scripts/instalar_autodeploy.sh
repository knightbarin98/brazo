#!/bin/bash
# =============================================================
#  Auto-deploy del brazo robótico en la Raspberry Pi.
#  Clona el repo, crea el entorno de conda, deja servidor.py corriendo como
#  servicio y revisa cada minuto si hay commits nuevos para bajarlos.
#
#  Requisitos: Raspberry Pi OS de 64 bits y conda (Miniforge) instalado para tu usuario.
#
#  Uso:
#    sudo bash instalar_autodeploy.sh <URL_REPO> [RAMA]
#
#  Ejemplos:
#    sudo bash instalar_autodeploy.sh https://github.com/usuario/brazo.git
#    sudo bash instalar_autodeploy.sh git@github.com:usuario/brazo.git main
#
#  Se puede volver a ejecutar para cambiar de rama o reparar la instalación.
# =============================================================
set -e

REPO_URL="${1:-}"
RAMA="${2:-main}"
ENTORNO="brazo-pi"
PUERTO_RED="8765"
INTERVALO="1min"          # cada cuánto revisa si hay cambios

if [ "$EUID" -ne 0 ]; then
  echo "Ejecútalo con sudo:  sudo bash $0 <URL_REPO> [RAMA]"
  exit 1
fi
if [ -z "$REPO_URL" ]; then
  echo "Falta la URL del repositorio."
  echo "Uso: sudo bash $0 <URL_REPO> [RAMA]"
  exit 1
fi

USUARIO="${SUDO_USER:-pi}"
HOME_USR="$(getent passwd "$USUARIO" | cut -d: -f6)"
DIR_APP="$HOME_USR/brazo"
SCRIPT_ACT="/usr/local/bin/brazo-actualizar.sh"
CONFIG="/etc/default/brazo-deploy"
LOG="$HOME_USR/brazo-deploy.log"
como_usuario() { runuser -u "$USUARIO" -- "$@"; }

echo "==> Usuario: $USUARIO | Repo: $REPO_URL | Rama: $RAMA"

# ---------- 1. conda ----------
CONDA="${CONDA:-}"
for candidato in "$HOME_USR/miniforge3/bin/conda" "$HOME_USR/mambaforge/bin/conda" \
                 "$HOME_USR/miniconda3/bin/conda" "$HOME_USR/anaconda3/bin/conda" /opt/conda/bin/conda; do
  [ -z "$CONDA" ] && [ -x "$candidato" ] && CONDA="$candidato"
done
if [ -z "$CONDA" ]; then
  echo "No encontré conda para $USUARIO. Instala Miniforge (como $USUARIO, sin sudo) y vuelve a ejecutar:"
  echo "  curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
  echo "  bash Miniforge3-Linux-aarch64.sh -b"
  echo "O indica dónde está:  sudo CONDA=/ruta/a/conda bash $0 $REPO_URL $RAMA"
  exit 1
fi
CONDA_BASE="$(dirname "$(dirname "$CONDA")")"
PYTHON="$CONDA_BASE/envs/$ENTORNO/bin/python"
echo "==> conda: $CONDA"

# ---------- 2. Dependencias del sistema ----------
echo "==> Instalando git..."
apt-get update -qq
apt-get install -y -qq git iproute2

# ---------- 3. Llave SSH si el repo es privado por SSH ----------
if [[ "$REPO_URL" == git@* ]]; then
  KEY="$HOME_USR/.ssh/id_ed25519"
  if [ ! -f "$KEY" ]; then
    echo "==> Generando llave SSH para $USUARIO..."
    como_usuario mkdir -p "$HOME_USR/.ssh"
    como_usuario ssh-keygen -t ed25519 -N "" -f "$KEY" -C "brazo-raspberry" -q
  fi
  como_usuario bash -c "ssh-keyscan -H github.com >> '$HOME_USR/.ssh/known_hosts' 2>/dev/null"
  echo ""
  echo "--------------------------------------------------------------"
  echo " Agrega esta llave en GitHub > tu repo > Settings > Deploy keys"
  echo " (sólo lectura: la Raspberry nunca necesita hacer push)"
  echo "--------------------------------------------------------------"
  cat "$KEY.pub"
  echo "--------------------------------------------------------------"
  read -rp "Presiona ENTER cuando la hayas agregado..."
fi

# ---------- 4. Clonar el repo ----------
# Las calibraciones (*.json) y token_remoto.txt no están en el repo: reset --hard no los toca.
if [ -d "$DIR_APP/.git" ]; then
  echo "==> El repo ya existe en $DIR_APP, se actualizará."
  como_usuario git -C "$DIR_APP" remote set-url origin "$REPO_URL"
  como_usuario git -C "$DIR_APP" fetch -q origin "$RAMA"
  como_usuario git -C "$DIR_APP" checkout -q -B "$RAMA" "origin/$RAMA"
else
  echo "==> Clonando en $DIR_APP..."
  como_usuario git clone -q -b "$RAMA" "$REPO_URL" "$DIR_APP"
fi

# ---------- 5. Entorno de conda ----------
if [ -x "$PYTHON" ]; then
  echo "==> Actualizando el entorno $ENTORNO..."
  como_usuario "$CONDA" env update -q -n "$ENTORNO" -f "$DIR_APP/environment-raspberry.yml" --prune
else
  echo "==> Creando el entorno $ENTORNO (tarda varios minutos)..."
  como_usuario "$CONDA" env create -q -f "$DIR_APP/environment-raspberry.yml"
fi

# ---------- 6. Permiso para el puerto serial del Arduino ----------
usermod -aG dialout "$USUARIO"

# ---------- 7. Servicio del brazo ----------
echo "==> Creando servicio brazo.service..."
cat > /etc/systemd/system/brazo.service <<EOF
[Unit]
Description=Brazo robotico (servidor.py)
After=network-online.target
Wants=network-online.target
# Si falla 5 veces en 2 minutos deja de reintentar: cada intento reinicia el Arduino.
StartLimitIntervalSec=120
StartLimitBurst=5

[Service]
User=$USUARIO
SupplementaryGroups=dialout
WorkingDirectory=$DIR_APP
ExecStart=$PYTHON -u $DIR_APP/servidor.py
Restart=on-failure
RestartSec=10
# SIGINT = Ctrl+C: servidor.py cierra las conexiones y deja el brazo donde está.
KillSignal=SIGINT
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

# ---------- 8. Script de actualización y su configuración ----------
echo "==> Instalando el script de actualización..."
cat > "$CONFIG" <<EOF
DIR_APP=$DIR_APP
RAMA=$RAMA
USUARIO=$USUARIO
CONDA=$CONDA
ENTORNO=$ENTORNO
PUERTO_RED=$PUERTO_RED
EOF
install -m 0755 "$DIR_APP/scripts/actualizar.sh" "$SCRIPT_ACT"

# ---------- 9. Timer que revisa cada minuto ----------
cat > /etc/systemd/system/brazo-deploy.service <<EOF
[Unit]
Description=Revisa si hay nueva versión del brazo en el repositorio
After=network-online.target

[Service]
Type=oneshot
EnvironmentFile=$CONFIG
ExecStart=$SCRIPT_ACT
StandardOutput=append:$LOG
StandardError=append:$LOG
EOF

cat > /etc/systemd/system/brazo-deploy.timer <<EOF
[Unit]
Description=Auto-deploy del brazo cada $INTERVALO

[Timer]
OnBootSec=30s
OnUnitActiveSec=$INTERVALO

[Install]
WantedBy=timers.target
EOF

# ---------- 10. Activar todo ----------
echo ""
echo "AVISO: al arrancar el servicio el Arduino se reinicia y el brazo va a HOME,"
echo "y lo mismo pasará en cada encendido de la Raspberry y en cada deploy."
if [ -t 0 ]; then
  read -rp "Despeja el área del brazo y presiona ENTER para continuar (Ctrl+C cancela)..."
fi
systemctl daemon-reload
systemctl enable --now brazo.service
systemctl enable --now brazo-deploy.timer

echo ""
echo "=============================================================="
echo " Listo. Cada push a '$RAMA' se instala en máximo $INTERVALO,"
echo " salvo mientras haya un operador conectado: entonces espera."
echo "=============================================================="
echo " Token para la app:         cat $DIR_APP/token_remoto.txt"
echo " Ver el servidor en vivo:   journalctl -u brazo -f"
echo " Ver historial de deploys:  tail -f $LOG"
echo " Reiniciar a mano:          sudo systemctl restart brazo"
echo " Detener el brazo:          sudo systemctl stop brazo"
echo " Pausar auto-deploy:        sudo systemctl stop brazo-deploy.timer"
echo " Calibrar con main.py:      primero sudo systemctl stop brazo"
echo "=============================================================="
