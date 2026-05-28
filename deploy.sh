#!/bin/bash
# =============================================================================
# deploy.sh — JONNY5 Raspberry Pi deploy script
#
# Copia il codice sul Pi, installa le dipendenze Python in un venv,
# installa i service systemd e riavvia tutti i servizi.
#
# Utilizzo:
#   ./deploy.sh [PI_HOST]            # default: jonny5@10.42.0.1
#   ./deploy.sh jonny5@192.168.1.50  # host personalizzato
#
# Prerequisiti sul PC:
#   - ssh / rsync installati
#   - accesso SSH al Pi (chiave o password)
#
# Prerequisiti sul Pi (una-tantum, se non già presenti):
#   - python3, python3-venv
#   - curl (per download MediaMTX automatico)
#
# MediaMTX viene scaricato automaticamente sul Pi se non già presente.
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Configurazione
# ---------------------------------------------------------------------------
PI_HOST="${1:-jonny5@10.42.0.1}"
REMOTE_DIR="/home/jonny5/raspberry5"
REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"

SERVICES=(
    jonny5-ws-teleop
    jonny5-spi-j5vr
    jonny5-https
    jonny5-https-443-proxy
    jonny5-captive-portal
    jonny5-video-init
    jonny5-mediamtx
)
TIMER="jonny5-video-fallback.timer"

# ---------------------------------------------------------------------------
# Colori
# ---------------------------------------------------------------------------
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
die()  { echo -e "${RED}[ERR]${NC} $*" >&2; exit 1; }

echo "========================================"
echo " JONNY5 deploy → ${PI_HOST}:${REMOTE_DIR}"
echo "========================================"

# ---------------------------------------------------------------------------
# 1. Sync codice (rsync esclude artefatti build)
# ---------------------------------------------------------------------------
echo ""
echo ">>> [1/7] Sync sorgenti..."
rsync -avz --delete \
    --exclude='.pio/' \
    --exclude='**/__pycache__/' \
    --exclude='**/*.pyc' \
    --exclude='.pytest_cache/' \
    --exclude='*.egg-info/' \
    --exclude='.DS_Store' \
    --exclude='Thumbs.db' \
    "${REPO_ROOT}/raspberry/" \
    "${PI_HOST}:${REMOTE_DIR}/"
ok "Sync completato."

# ---------------------------------------------------------------------------
# 2. Sync web assets
# ---------------------------------------------------------------------------
echo ""
echo ">>> [2/7] Sync web assets..."
rsync -avz --delete \
    --exclude='.DS_Store' \
    --exclude='node_modules/' \
    "${REPO_ROOT}/web/" \
    "${PI_HOST}:${REMOTE_DIR}/web/"
ok "Web assets sincronizzati."

# ---------------------------------------------------------------------------
# 3. Sync scripts
# ---------------------------------------------------------------------------
echo ""
echo ">>> [3/7] Sync scripts..."
rsync -avz \
    "${REPO_ROOT}/scripts/" \
    "${PI_HOST}:${REMOTE_DIR}/scripts/"
ssh "${PI_HOST}" "chmod +x ${REMOTE_DIR}/scripts/*.sh"
ok "Scripts sincronizzati."

# ---------------------------------------------------------------------------
# 4. MediaMTX binary (scarica solo se non già presente)
# ---------------------------------------------------------------------------
echo ""
echo ">>> [4/7] Verifica MediaMTX..."
ssh "${PI_HOST}" bash <<'EOF'
set -e

MEDIAMTX_BIN=""
if   [ -x "/home/jonny5/mediamtx" ] && file "/home/jonny5/mediamtx" 2>/dev/null | grep -q ELF; then
    MEDIAMTX_BIN="/home/jonny5/mediamtx"
elif [ -x "/home/jonny5/mediamtx/mediamtx" ]; then
    MEDIAMTX_BIN="/home/jonny5/mediamtx/mediamtx"
fi

if [ -n "$MEDIAMTX_BIN" ]; then
    echo "  MediaMTX già presente: ${MEDIAMTX_BIN}"
    exit 0
fi

echo "  MediaMTX non trovato — scarico ultima release da GitHub..."

# Rileva architettura
ARCH=$(uname -m)
case "$ARCH" in
    aarch64|arm64) MEDIAMTX_ARCH="linux_arm64v8" ;;
    armv7l)        MEDIAMTX_ARCH="linux_armv7" ;;
    armv6l)        MEDIAMTX_ARCH="linux_armv6" ;;
    x86_64)        MEDIAMTX_ARCH="linux_amd64" ;;
    *)             echo "  WARN: architettura ${ARCH} non riconosciuta, provo linux_arm64v8"; MEDIAMTX_ARCH="linux_arm64v8" ;;
esac

echo "  Architettura rilevata: ${ARCH} → ${MEDIAMTX_ARCH}"

# Recupera URL ultima release da GitHub API
DOWNLOAD_URL=$(curl -sf https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
    | grep "browser_download_url" \
    | grep "${MEDIAMTX_ARCH}\.tar\.gz" \
    | head -1 \
    | cut -d '"' -f 4)

if [ -z "$DOWNLOAD_URL" ]; then
    echo "  ERR: impossibile recuperare URL download MediaMTX. Scaricalo manualmente da:"
    echo "  https://github.com/bluenviron/mediamtx/releases"
    exit 1
fi

echo "  Download: ${DOWNLOAD_URL}"
TMP_DIR=$(mktemp -d)
curl -L --progress-bar "${DOWNLOAD_URL}" -o "${TMP_DIR}/mediamtx.tar.gz"
tar -xzf "${TMP_DIR}/mediamtx.tar.gz" -C "${TMP_DIR}"
mv "${TMP_DIR}/mediamtx" "/home/jonny5/mediamtx"
chmod +x "/home/jonny5/mediamtx"
rm -rf "${TMP_DIR}"

echo "  MediaMTX installato in /home/jonny5/mediamtx"
VERSION=$(/home/jonny5/mediamtx --version 2>/dev/null | head -1 || echo "versione non disponibile")
echo "  Versione: ${VERSION}"
EOF
ok "MediaMTX pronto."

# ---------------------------------------------------------------------------
# 5. Python venv + dipendenze
# ---------------------------------------------------------------------------
echo ""
echo ">>> [5/7] Aggiornamento venv Python..."
ssh "${PI_HOST}" bash <<EOF
set -e
VENV="${REMOTE_DIR}/.venv"
if [ ! -d "\$VENV" ]; then
    echo "  Creazione venv..."
    python3 -m venv "\$VENV"
fi
echo "  pip install -r requirements-controller.txt..."
"\$VENV/bin/pip" install --quiet --upgrade pip
"\$VENV/bin/pip" install --quiet -r "${REMOTE_DIR}/requirements-controller.txt"
echo "  Done."
EOF
ok "Venv aggiornato."

# ---------------------------------------------------------------------------
# 6. Certificati TLS (genera solo se non già presenti)
# ---------------------------------------------------------------------------
echo ""
echo ">>> [6/7] Verifica certificati TLS..."
ssh "${PI_HOST}" bash <<'EOF'
set -e
TLS_DIR="/home/jonny5/raspberry5/config_runtime/tls"
CERT="${TLS_DIR}/webrtc.crt"
KEY="${TLS_DIR}/webrtc.key"

if [ -f "$CERT" ] && [ -f "$KEY" ]; then
    EXPIRY=$(openssl x509 -enddate -noout -in "$CERT" 2>/dev/null | cut -d= -f2 || echo "?")
    echo "  Certificato già presente (scade: ${EXPIRY}) — skip."
    exit 0
fi

echo "  Certificati non trovati — generazione self-signed..."
mkdir -p "$TLS_DIR"

# SAN con IP e hostname per compatibilità browser/WebRTC
openssl req -x509 -nodes -newkey rsa:2048 -days 3650 \
    -keyout "$KEY" \
    -out "$CERT" \
    -subj "/CN=jonny5.local/O=JONNY5/C=IT" \
    -addext "subjectAltName=IP:10.42.0.1,DNS:jonny5.local,DNS:localhost"

chmod 640 "$KEY" "$CERT"
echo "  Certificato generato (valido 10 anni):"
openssl x509 -subject -enddate -noout -in "$CERT"
echo ""
echo "  NOTA: certificato self-signed — il browser mostrerà un avviso."
echo "  Per accettarlo una volta su Quest/Android:"
echo "    https://10.42.0.1/ → 'Avanzate' → 'Procedi comunque'"
EOF
ok "Certificati TLS pronti."

# ---------------------------------------------------------------------------
# 7. Systemd services
# ---------------------------------------------------------------------------
echo ""
echo ">>> [7/7] Installazione e riavvio servizi systemd..."

# Copia service files
ssh "${PI_HOST}" bash <<EOF
set -e
for svc in ${SERVICES[@]} ${TIMER}; do
    src="${REMOTE_DIR}/systemd/\${svc}.service"
    # timer ha estensione diversa
    [ -f "\${src}" ] || src="${REMOTE_DIR}/systemd/\${svc}"
    [ -f "\${src}" ] || { echo "  SKIP: \${svc} (file non trovato)"; continue; }
    sudo cp "\${src}" "/etc/systemd/system/"
    echo "  Installato: \${svc}"
done

# NetworkManager dnsmasq captive portal
DNSMASQ_CONF="${REMOTE_DIR}/networkmanager/dnsmasq-shared.d/jonny5-captive-portal.conf"
if [ -f "\${DNSMASQ_CONF}" ]; then
    sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d/
    sudo cp "\${DNSMASQ_CONF}" /etc/NetworkManager/dnsmasq-shared.d/
    echo "  Installato: dnsmasq captive portal conf"
fi

sudo systemctl daemon-reload
EOF

# Riavvia servizi
ssh "${PI_HOST}" bash <<'EOF'
set -e
SERVICES=(jonny5-ws-teleop jonny5-spi-j5vr jonny5-https jonny5-https-443-proxy jonny5-captive-portal)
for svc in "${SERVICES[@]}"; do
    if systemctl is-enabled --quiet "${svc}" 2>/dev/null; then
        sudo systemctl restart "${svc}" && echo "  Riavviato: ${svc}"
    else
        sudo systemctl enable --now "${svc}" && echo "  Abilitato+avviato: ${svc}"
    fi
done

# Video init: oneshot, non si riavvia come gli altri
for svc in jonny5-video-init jonny5-video-fallback.timer; do
    if ! systemctl is-enabled --quiet "${svc}" 2>/dev/null; then
        sudo systemctl enable "${svc}" && echo "  Abilitato: ${svc}"
    fi
done

echo ""
echo "--- Stato servizi ---"
for svc in jonny5-ws-teleop jonny5-spi-j5vr jonny5-https jonny5-captive-portal jonny5-mediamtx; do
    STATUS=$(systemctl is-active "${svc}" 2>/dev/null || echo "inattivo")
    echo "  ${svc}: ${STATUS}"
done
EOF

ok "Servizi aggiornati."

# ---------------------------------------------------------------------------
# Fine
# ---------------------------------------------------------------------------
echo ""
echo -e "${GREEN}========================================"
echo " Deploy completato su ${PI_HOST}"
echo -e "========================================${NC}"
echo ""
echo "Accesso dashboard:  https://10.42.0.1/"
echo "WebSocket teleop:   wss://10.42.0.1:8557/"
echo ""
echo "Log in tempo reale:"
echo "  ssh ${PI_HOST} 'journalctl -fu jonny5-ws-teleop'"
echo "  ssh ${PI_HOST} 'journalctl -fu jonny5-spi-j5vr'"
