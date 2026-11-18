#!/bin/bash
# =============================================================
#  Deshace todo lo que instaló el auto-deploy del brazo, para empezar de cero.
#  Sirve tanto para scripts/instalar_autodeploy.sh como para el script original
#  (el que usaba ~/brazo-venv y requirements.txt).
#
#  Uso (lo más cómodo:  python main.py desinstalar):
#    sudo bash desinstalar.sh [--si] [--borrar-repo] [--borrar-llave]
#
#  Siempre: detiene y borra los servicios, el script de actualización, su configuración,
#  el registro, la marca scripts/.servicio_instalado y los entornos brazo-gemelo y brazo-pi.
#  Pregunta antes de: borrar la carpeta del repo (con respaldo .tar.gz), la llave SSH y el
#  entorno 'brazo'. --si responde que sí a todo; --borrar-repo / --borrar-llave lo hacen sin preguntar.
#  Los grupos de usuario sólo se quitan si consta que los agregó el instalador.
#  Las calibraciones (*.json) y el token viven dentro del repo: sólo se van si se borra el repo,
#  y entonces quedan en el respaldo.
# =============================================================
set -u

RAIZ_SISTEMA="${BRAZO_RAIZ:-}"          # sólo para pruebas: prefijo para /etc, /usr y /run
SI=0; BORRAR_REPO=0; BORRAR_LLAVE=0
for arg in "$@"; do
  case "$arg" in
    --si|-y) SI=1 ;;
    --borrar-repo) BORRAR_REPO=1 ;;
    --borrar-llave) BORRAR_LLAVE=1 ;;
    *) echo "Opción desconocida: $arg"; echo "Uso: sudo bash $0 [--si] [--borrar-repo] [--borrar-llave]"; exit 1 ;;
  esac
done

if [ "$EUID" -ne 0 ] && [ -z "$RAIZ_SISTEMA" ]; then
  echo "Ejecútalo con sudo:  sudo bash $0   (o:  python main.py desinstalar)"
  exit 1
fi

USUARIO="${SUDO_USER:-pi}"
HOME_USR="${BRAZO_HOME:-$(getent passwd "$USUARIO" | cut -d: -f6)}"
CONFIG="$RAIZ_SISTEMA/etc/default/brazo-deploy"
UNIDADES="$RAIZ_SISTEMA/etc/systemd/system"
DIR_APP="$HOME_USR/brazo"; CONDA=""; GRUPOS_AGREGADOS=""; LLAVE_GENERADA="no"
if [ -f "$CONFIG" ]; then
  # shellcheck disable=SC1090
  . "$CONFIG"
fi

como_usuario() { runuser -u "$USUARIO" -- "$@"; }
preguntar() {
  [ "$SI" = 1 ] && return 0
  [ -t 0 ] || return 1
  read -rp "$1 [s/N] " r
  [ "$r" = "s" ] || [ "$r" = "S" ]
}

# Si este script vive dentro del repo y el repo se va a borrar, primero se copia fuera.
if [ -z "${BRAZO_DESINSTALAR_COPIA:-}" ]; then
  case "$(cd "$(dirname "$0")" && pwd -P)/" in
    "$(cd "$DIR_APP" 2>/dev/null && pwd -P)"/*)
      copia="$(mktemp /tmp/brazo-desinstalar.XXXXXX)"
      cp "$0" "$copia"
      BRAZO_DESINSTALAR_COPIA=1 exec bash "$copia" "$@" ;;
  esac
fi

echo "==> Usuario: $USUARIO | Repo: $DIR_APP"

# ---------- 1. Servicios ----------
echo "==> Deteniendo y borrando los servicios (el brazo se queda donde esté)..."
for unidad in brazo-deploy.timer brazo-deploy.service brazo.service; do
  systemctl stop "$unidad" 2>/dev/null
  systemctl disable "$unidad" 2>/dev/null
  rm -f "$UNIDADES/$unidad"
done
systemctl daemon-reload 2>/dev/null
systemctl reset-failed brazo.service brazo-deploy.service 2>/dev/null

# ---------- 2. Script de actualización, configuración, marcas y registro ----------
rm -f "$RAIZ_SISTEMA/usr/local/bin/brazo-actualizar.sh" "$CONFIG" \
      "$RAIZ_SISTEMA/run/brazo-deploy.pendiente" "$RAIZ_SISTEMA/run/brazo-deploy.cambios-locales" \
      "$HOME_USR/brazo-deploy.log" "$DIR_APP/scripts/.servicio_instalado"

# ---------- 3. Lo que dejaba el script original ----------
if [ -d "$HOME_USR/brazo-venv" ]; then
  echo "==> Borrando el entorno virtual del script original ($HOME_USR/brazo-venv)..."
  rm -rf "$HOME_USR/brazo-venv"
fi
git config --system --unset-all safe.directory "^$DIR_APP\$" 2>/dev/null

# ---------- 4. Grupos ----------
for grupo in $GRUPOS_AGREGADOS; do
  echo "==> Quitando a $USUARIO del grupo $grupo (lo había agregado el instalador)..."
  gpasswd -d "$USUARIO" "$grupo" >/dev/null 2>&1
done
if [ -z "$GRUPOS_AGREGADOS" ]; then
  echo "    Grupos (dialout, gpio, i2c): sin cambios; no consta que los agregara el instalador."
fi

# ---------- 5. Entornos de conda ----------
if [ -z "$CONDA" ]; then
  for candidato in "$HOME_USR/miniforge3/bin/conda" "$HOME_USR/mambaforge/bin/conda" \
                   "$HOME_USR/miniconda3/bin/conda" "$HOME_USR/anaconda3/bin/conda" /opt/conda/bin/conda; do
    [ -z "$CONDA" ] && [ -x "$candidato" ] && CONDA="$candidato"
  done
fi
if [ -n "$CONDA" ]; then
  ENVS="$(dirname "$(dirname "$CONDA")")/envs"
  for entorno in brazo-gemelo brazo-pi brazo; do
    [ -d "$ENVS/$entorno" ] || continue
    if [ "$entorno" = "brazo" ] && ! preguntar "¿Borrar también el entorno 'brazo' (puede ser el de desarrollo de alguien)?"; then
      continue
    fi
    echo "==> Borrando el entorno de conda '$entorno'..."
    como_usuario "$CONDA" env remove -y -q -n "$entorno" >/dev/null || echo "    No se pudo borrar '$entorno'."
  done
else
  echo "    No encontré conda: no hay entornos que borrar."
fi

# ---------- 6. Llave SSH generada para GitHub ----------
LLAVE="$HOME_USR/.ssh/id_ed25519"
if [ -f "$LLAVE.pub" ] && { [ "$LLAVE_GENERADA" = "si" ] || grep -q "brazo-raspberry" "$LLAVE.pub"; }; then
  if [ "$BORRAR_LLAVE" = 1 ] || preguntar "¿Borrar la llave SSH que generó el instalador ($LLAVE)?"; then
    rm -f "$LLAVE" "$LLAVE.pub"
    echo "    Llave borrada. Quítala también en GitHub > tu repo > Settings > Deploy keys."
  fi
fi

# ---------- 7. Carpeta del repo ----------
if [ -d "$DIR_APP" ]; then
  if [ "$BORRAR_REPO" = 1 ] || preguntar "¿Borrar también $DIR_APP? Antes se guarda un respaldo completo"; then
    if [ -d "$DIR_APP/.git" ]; then
      pendientes="$(como_usuario git -C "$DIR_APP" status --porcelain 2>/dev/null | wc -l | xargs)"
      echo "    Cambios sin commit (o archivos nuevos) en el repo: $pendientes"
    fi
    respaldo="$HOME_USR/brazo-respaldo-$(date '+%Y%m%d-%H%M%S').tar.gz"
    if tar -czf "$respaldo" -C "$(dirname "$DIR_APP")" "$(basename "$DIR_APP")"; then
      chown "$USUARIO" "$respaldo" 2>/dev/null
      rm -rf "$DIR_APP"
      echo "==> Repo borrado. Respaldo (calibraciones, token y cambios incluidos): $respaldo"
    else
      echo "    No se pudo crear el respaldo: el repo NO se borró."
    fi
  else
    echo "    El repo se conserva en $DIR_APP (con sus calibraciones y su token)."
  fi
fi

echo ""
echo "=============================================================="
echo " Listo: no queda ningún servicio del brazo."
echo " Para empezar de cero:  python main.py instalar"
echo "=============================================================="
