#!/bin/bash
# =============================================================
#  Auto-deploy del brazo: lo ejecuta brazo-deploy.timer cada minuto, como root.
#  Si hay commits nuevos en la rama, los baja, actualiza el entorno de conda si
#  environment-raspberry.yml cambió y reinicia servidor.py.
#
#  La configuración la escribe instalar_autodeploy.sh en /etc/default/brazo-deploy.
#  No lo edites en /usr/local/bin: se reemplaza con la versión del repo en cada deploy.
# =============================================================
set -u
: "${DIR_APP:?}" "${RAMA:?}" "${USUARIO:?}" "${CONDA:?}"
ENTORNO="${ENTORNO:-brazo-pi}"
PUERTO_RED="${PUERTO_RED:-8765}"
DESTINO_SCRIPT="${DESTINO_SCRIPT:-/usr/local/bin/brazo-actualizar.sh}"
MARCA_PENDIENTE="${MARCA_PENDIENTE:-/run/brazo-deploy.pendiente}"

log() { echo "$(date '+%F %T') - $*"; }
# git y conda corren como el usuario: así los archivos nunca quedan como root.
como_usuario() { runuser -u "$USUARIO" -- "$@"; }

cd "$DIR_APP" || { log "No existe $DIR_APP"; exit 1; }
como_usuario git fetch -q origin "$RAMA" || { log "Sin conexión con el repositorio"; exit 0; }
LOCAL=$(como_usuario git rev-parse HEAD)
REMOTO=$(como_usuario git rev-parse "origin/$RAMA")
[ "$LOCAL" = "$REMOTO" ] && exit 0

# Reiniciar el servidor reinicia el Arduino: el brazo se mueve y la garra suelta lo que tenga.
# Mientras alguien lo maneja, el deploy espera.
if [ -n "$(ss -Htn state established "( sport = :$PUERTO_RED )" 2>/dev/null)" ]; then
  if [ "$(cat "$MARCA_PENDIENTE" 2>/dev/null)" != "$REMOTO" ]; then
    log "Nueva versión ${REMOTO:0:7}: se instalará cuando se desconecte el operador."
    echo "$REMOTO" > "$MARCA_PENDIENTE"
  fi
  exit 0
fi
rm -f "$MARCA_PENDIENTE"

log "Nueva versión ${REMOTO:0:7} (antes ${LOCAL:0:7})"
ENTORNO_ANTES=$(como_usuario git rev-parse -q --verify HEAD:environment-raspberry.yml 2>/dev/null)
# reset --hard no toca los archivos sin seguimiento: calibraciones y token se conservan.
como_usuario git reset -q --hard "origin/$RAMA"
ENTORNO_DESPUES=$(como_usuario git rev-parse -q --verify HEAD:environment-raspberry.yml 2>/dev/null)

if [ "$ENTORNO_ANTES" != "$ENTORNO_DESPUES" ]; then
  log "environment-raspberry.yml cambió: actualizando el entorno $ENTORNO (puede tardar)..."
  if como_usuario "$CONDA" env update -q -n "$ENTORNO" -f environment-raspberry.yml --prune; then
    log "Entorno actualizado"
  else
    log "ERROR al actualizar el entorno: revisa con 'conda env update -n $ENTORNO -f environment-raspberry.yml'"
  fi
fi

# El propio script también se actualiza. install crea un archivo nuevo, así que el que
# está corriendo ahora no se altera.
if [ -f scripts/actualizar.sh ] && ! cmp -s scripts/actualizar.sh "$DESTINO_SCRIPT"; then
  install -m 0755 scripts/actualizar.sh "$DESTINO_SCRIPT" && log "Script de actualización renovado"
fi

systemctl restart brazo && log "Servidor reiniciado con ${REMOTO:0:7}" || log "ERROR al reiniciar: journalctl -u brazo"
