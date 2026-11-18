#!/bin/bash
# =============================================================
#  Auto-deploy del brazo robótico en la Raspberry Pi.
#  Clona el repo, crea el entorno de conda, deja el servidor corriendo como
#  servicio y revisa cada minuto si hay commits nuevos para bajarlos.
#  Lo más cómodo es lanzarlo con:  python main.py instalar
#  Al terminar deja scripts/.servicio_instalado; scripts/desinstalar.sh lo deshace todo.
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
ENTORNO="brazo-gemelo"
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
GRUPOS_AGREGADOS=""
LLAVE_GENERADA="no"
if [ -f "$CONFIG" ]; then             # reinstalación: lo que ya agregó una instalación anterior sigue siendo nuestro
  GRUPOS_AGREGADOS="$(. "$CONFIG"; echo "${GRUPOS_AGREGADOS:-}")"
  LLAVE_GENERADA="$(. "$CONFIG"; echo "${LLAVE_GENERADA:-no}")"
fi
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
    LLAVE_GENERADA="si"
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
  # Si se trabajó en la Raspberry, no se pisa nada: cambios sin commit o commits sin subir detienen todo.
  SIN_SUBIR=0
  for ref in HEAD "refs/heads/$RAMA"; do
    if como_usuario git -C "$DIR_APP" rev-parse -q --verify "$ref" >/dev/null; then
      SIN_SUBIR=$((SIN_SUBIR + $(como_usuario git -C "$DIR_APP" rev-list --count "origin/$RAMA..$ref")))
    fi
  done
  if [ -n "$(como_usuario git -C "$DIR_APP" status --porcelain --untracked-files=no)" ] || [ "$SIN_SUBIR" -gt 0 ]; then
    echo "ERROR: $DIR_APP tiene cambios sin commit o commits que no están en origin/$RAMA."
    echo "Súbelos (git push) o guárdalos antes. 'python main.py desinstalar' hace un respaldo."
    exit 1
  fi
  como_usuario git -C "$DIR_APP" checkout -q -B "$RAMA" "origin/$RAMA"
else
  echo "==> Clonando en $DIR_APP..."
  como_usuario git clone -q -b "$RAMA" "$REPO_URL" "$DIR_APP"
fi

# ---------- 5. Entorno de conda (el mismo environment.yml de todas las máquinas) ----------
if [ -x "$PYTHON" ]; then
  echo "==> Actualizando el entorno $ENTORNO..."
  como_usuario "$CONDA" env update -q -n "$ENTORNO" -f "$DIR_APP/environment.yml" --prune
else
  echo "==> Creando el entorno $ENTORNO (tarda varios minutos)..."
  como_usuario "$CONDA" env create -q -n "$ENTORNO" -f "$DIR_APP/environment.yml"
fi
# main.py compara esta huella para saber que el entorno corresponde a environment.yml
como_usuario bash -c "sha256sum '$DIR_APP/environment.yml' | cut -d' ' -f1 > '$CONDA_BASE/envs/$ENTORNO/.brazo-environment.sha256'"
for viejo in brazo-pi brazo; do
  if [ -d "$CONDA_BASE/envs/$viejo" ]; then
    echo "    (el entorno viejo '$viejo' ya no se usa; 'python main.py desinstalar' lo borra)"
  fi
done

# ---------- 6. Permiso para el puerto serial del Arduino ----------
# Se anota si el grupo lo agregamos nosotros: así desinstalar.sh sólo quita lo que puso.
if ! id -nG "$USUARIO" | tr ' ' '\n' | grep -qx dialout; then
  usermod -aG dialout "$USUARIO"
  GRUPOS_AGREGADOS="$(echo "$GRUPOS_AGREGADOS dialout" | xargs)"
fi

# ---------- 7. Servicio del brazo ----------
echo "==> Creando servicio brazo.service..."
cat > /etc/systemd/system/brazo.service <<EOF
[Unit]
Description=Brazo robotico (python main.py servidor)
After=network-online.target
Wants=network-online.target
# Si falla 5 veces en 2 minutos deja de reintentar: cada intento reinicia el Arduino.
StartLimitIntervalSec=120
StartLimitBurst=5

[Service]
User=$USUARIO
SupplementaryGroups=dialout
WorkingDirectory=$DIR_APP
# BRAZO_SERVICIO: main.py sabe que es el servicio y no se bloquea a sí mismo por "Arduino en uso".
Environment=BRAZO_SERVICIO=1
ExecStart=$PYTHON -u $DIR_APP/main.py servidor
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
REPO_URL=$REPO_URL
GRUPOS_AGREGADOS="$GRUPOS_AGREGADOS"
LLAVE_GENERADA=$LLAVE_GENERADA
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

# Marca de instalación: si existe, el servicio y el auto-deploy quedaron instalados y activos.
# No está en el repo (.gitignore): el auto-deploy no la borra; desinstalar.sh sí.
como_usuario bash -c "cat > '$DIR_APP/scripts/.servicio_instalado'" <<EOF
FECHA=$(date '+%F %T')
USUARIO=$USUARIO
RAMA=$RAMA
REPO_URL=$REPO_URL
ENTORNO=$ENTORNO
CONDA=$CONDA
DIR_APP=$DIR_APP
EOF

echo ""
echo "=============================================================="
echo " Listo. Cada push a '$RAMA' se instala en máximo $INTERVALO,"
echo " salvo mientras haya un operador conectado: entonces espera."
echo "=============================================================="
echo " Estado:                    python main.py estado"
echo " Token para la app:         cat $DIR_APP/token_remoto.txt"
echo " Ver el servidor en vivo:   journalctl -u brazo -f"
echo " Ver historial de deploys:  tail -f $LOG"
echo " Reiniciar a mano:          sudo systemctl restart brazo"
echo " Detener el brazo:          sudo systemctl stop brazo"
echo " Pausar auto-deploy:        sudo systemctl stop brazo-deploy.timer"
echo " Calibrar:                  sudo systemctl stop brazo  y luego  python main.py consola"
echo " Deshacer todo:             python main.py desinstalar"
echo "=============================================================="
