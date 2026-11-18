#!/bin/bash
# =============================================================
#  Auto-deploy del brazo: lo ejecuta brazo-deploy.timer cada minuto, como root.
#  Si hay commits nuevos en la rama, los baja, actualiza el entorno de conda si
#  environment.yml cambió y reinicia el servidor (python main.py servidor).
#
#  La configuración la escribe instalar_autodeploy.sh en /etc/default/brazo-deploy.
#  No lo edites en /usr/local/bin: se reemplaza con la versión del repo en cada deploy.
# =============================================================
set -u
: "${DIR_APP:?}" "${RAMA:?}" "${USUARIO:?}" "${CONDA:?}"
ENTORNO="${ENTORNO:-brazo-gemelo}"
PUERTO_RED="${PUERTO_RED:-8765}"
DESTINO_SCRIPT="${DESTINO_SCRIPT:-/usr/local/bin/brazo-actualizar.sh}"
MARCA_PENDIENTE="${MARCA_PENDIENTE:-/run/brazo-deploy.pendiente}"
MARCA_LOCAL="${MARCA_LOCAL:-/run/brazo-deploy.cambios-locales}"

log() { echo "$(date '+%F %T') - $*"; }
# git y conda corren como el usuario: así los archivos nunca quedan como root.
como_usuario() { runuser -u "$USUARIO" -- "$@"; }
huella() { if command -v sha256sum >/dev/null; then sha256sum "$1"; else shasum -a 256 "$1"; fi | cut -d' ' -f1; }
# Una instalación anterior usaba environment-raspberry.yml; desde la unificación es environment.yml.
archivo_entorno() { if [ -f environment.yml ]; then echo environment.yml; else echo environment-raspberry.yml; fi; }

cd "$DIR_APP" || { log "No existe $DIR_APP"; exit 1; }
como_usuario git fetch -q origin "$RAMA" || { log "Sin conexión con el repositorio"; exit 0; }
LOCAL=$(como_usuario git rev-parse HEAD)
REMOTO=$(como_usuario git rev-parse "origin/$RAMA")
[ "$LOCAL" = "$REMOTO" ] && exit 0

# Si alguien trabajó en la Raspberry, el deploy no pisa su trabajo: espera a que lo suba o lo guarde.
CAMBIOS=$(como_usuario git status --porcelain --untracked-files=no)
SIN_SUBIR=$(como_usuario git rev-list --count "origin/$RAMA..HEAD")
if [ -n "$CAMBIOS" ] || [ "$SIN_SUBIR" -gt 0 ]; then
  if [ "$(cat "$MARCA_LOCAL" 2>/dev/null)" != "$REMOTO" ]; then
    log "Nueva versión ${REMOTO:0:7}, pero $DIR_APP tiene cambios sin commit o $SIN_SUBIR commit(s) sin subir: no se actualiza. Súbelos (git push) o descártalos."
    echo "$REMOTO" > "$MARCA_LOCAL"
  fi
  exit 0
fi
rm -f "$MARCA_LOCAL"

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
ARCHIVO_ANTES=$(archivo_entorno)
ENTORNO_ANTES=$(como_usuario git rev-parse -q --verify "HEAD:$ARCHIVO_ANTES" 2>/dev/null)
# reset --hard no toca los archivos sin seguimiento: calibraciones, token y scripts/.servicio_instalado se conservan.
como_usuario git reset -q --hard "origin/$RAMA"
ARCHIVO=$(archivo_entorno)
ENTORNO_DESPUES=$(como_usuario git rev-parse -q --verify "HEAD:$ARCHIVO" 2>/dev/null)

if [ "$ARCHIVO" != "$ARCHIVO_ANTES" ] || [ "$ENTORNO_ANTES" != "$ENTORNO_DESPUES" ]; then
  log "$ARCHIVO cambió: actualizando el entorno $ENTORNO (puede tardar)..."
  if como_usuario "$CONDA" env update -q -n "$ENTORNO" -f "$ARCHIVO" --prune; then
    PREFIJO="$(dirname "$(dirname "$CONDA")")/envs/$ENTORNO"
    huella "$ARCHIVO" | como_usuario tee "$PREFIJO/.brazo-environment.sha256" >/dev/null
    log "Entorno actualizado"
  else
    log "ERROR al actualizar el entorno: revisa con 'python main.py entorno'"
  fi
fi

# El propio script también se actualiza. install crea un archivo nuevo, así que el que
# está corriendo ahora no se altera.
if [ -f scripts/actualizar.sh ] && ! cmp -s scripts/actualizar.sh "$DESTINO_SCRIPT"; then
  install -m 0755 scripts/actualizar.sh "$DESTINO_SCRIPT" && log "Script de actualización renovado"
fi

systemctl restart brazo && log "Servidor reiniciado con ${REMOTO:0:7}" || log "ERROR al reiniciar: journalctl -u brazo"
