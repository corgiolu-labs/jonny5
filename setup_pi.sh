#!/bin/bash
# =============================================================================
# setup_pi.sh — JONNY5 Raspberry Pi first-time setup
#
# Da eseguire UNA SOLA VOLTA su un Raspberry Pi OS fresco (Trixie/Debian 13),
# PRIMA del primo deploy.sh.
#
# Cosa fa:
#   1. Aggiorna il sistema
#   2. Installa le dipendenze di sistema (python3-venv, openssl, rsync, curl...)
#   3. Crea l'utente jonny5 con sudo passwordless
#   4. Abilita SPI (richiesto da jonny5-spi-j5vr)
#   5. Configura hotspot WiFi su 10.42.0.1 via NetworkManager
#   6. Abilita hotspot all'avvio
#
# Utilizzo (eseguire direttamente sul Pi come utente con sudo):
#   bash setup_pi.sh [WIFI_SSID] [WIFI_PASSWORD]
#
# Esempio:
#   bash setup_pi.sh JONNY5 robot1234
#
# Dopo il completamento: esegui deploy.sh dal PC.
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Parametri
# ---------------------------------------------------------------------------
WIFI_SSID="${1:-JONNY5}"
WIFI_PASSWORD="${2:-jonny5robot}"
JONNY5_USER="jonny5"
JONNY5_IP="10.42.0.1"
REMOTE_DIR="/home/${JONNY5_USER}/raspberry5"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ---------------------------------------------------------------------------
# Colori
# ---------------------------------------------------------------------------
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'
ok()    { echo -e "${GREEN}[OK]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
die()   { echo -e "${RED}[ERR]${NC} $*" >&2; exit 1; }
step()  { echo -e "\n${BOLD}>>> $*${NC}"; }

# Deve girare come root o con sudo
if [ "$EUID" -ne 0 ]; then
    die "Eseguire con sudo: sudo bash setup_pi.sh"
fi

echo "=============================================="
echo " JONNY5 — Setup Raspberry Pi (prima installazione)"
echo " SSID hotspot : ${WIFI_SSID}"
echo " Password     : ${WIFI_PASSWORD}"
echo " IP fisso     : ${JONNY5_IP}"
echo " Utente       : ${JONNY5_USER}"
echo "=============================================="
echo ""
read -rp "Continuare? [y/N] " CONFIRM
[[ "$CONFIRM" =~ ^[Yy]$ ]] || { echo "Annullato."; exit 0; }

# ---------------------------------------------------------------------------
# 1. Aggiornamento sistema + dipendenze
# ---------------------------------------------------------------------------
step "[1/5] Aggiornamento sistema e installazione dipendenze..."

apt-get update -qq
apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-pip \
    openssl \
    rsync \
    curl \
    git \
    i2c-tools \
    network-manager \
    2>/dev/null

ok "Dipendenze installate."

# ---------------------------------------------------------------------------
# 2. Utente jonny5
# ---------------------------------------------------------------------------
step "[2/5] Configurazione utente '${JONNY5_USER}'..."

if id "${JONNY5_USER}" &>/dev/null; then
    warn "Utente '${JONNY5_USER}' già esistente — skip creazione."
else
    adduser --disabled-password --gecos "JONNY5 Robot" "${JONNY5_USER}"
    ok "Utente '${JONNY5_USER}' creato."
fi

# Sudo passwordless (richiesto da deploy.sh per systemctl)
SUDOERS_FILE="/etc/sudoers.d/${JONNY5_USER}"
echo "${JONNY5_USER} ALL=(ALL) NOPASSWD: ALL" > "${SUDOERS_FILE}"
chmod 440 "${SUDOERS_FILE}"
ok "Sudo passwordless configurato per '${JONNY5_USER}'."

# Crea directory destinazione deploy
mkdir -p "${REMOTE_DIR}"
chown -R "${JONNY5_USER}:${JONNY5_USER}" "/home/${JONNY5_USER}"
ok "Directory ${REMOTE_DIR} pronta."

# Gruppi utili (SPI, I2C, dialout per UART, video per camera)
for GRP in spi i2c dialout video gpio; do
    if getent group "${GRP}" &>/dev/null; then
        usermod -aG "${GRP}" "${JONNY5_USER}" 2>/dev/null && ok "  Aggiunto al gruppo: ${GRP}"
    fi
done

# ---------------------------------------------------------------------------
# 3. Abilita SPI
# ---------------------------------------------------------------------------
step "[3/5] Abilitazione interfacce hardware (SPI + camere + UART)..."

if raspi-config nonint do_spi 0 2>/dev/null; then
    ok "SPI abilitato via raspi-config."
else
    # Fallback manuale su /boot/config.txt o /boot/firmware/config.txt
    BOOT_CFG=""
    [ -f "/boot/firmware/config.txt" ] && BOOT_CFG="/boot/firmware/config.txt"
    [ -f "/boot/config.txt" ]          && BOOT_CFG="/boot/config.txt"

    if [ -n "$BOOT_CFG" ]; then
        if grep -q "^dtparam=spi=on" "${BOOT_CFG}"; then
            ok "SPI già abilitato in ${BOOT_CFG}."
        else
            echo "dtparam=spi=on" >> "${BOOT_CFG}"
            ok "SPI abilitato in ${BOOT_CFG} (richiede reboot)."
        fi
    else
        warn "Impossibile abilitare SPI automaticamente. Abilitare manualmente con raspi-config."
    fi
fi

# --- Camere (2x IMX708) + UART verso STM32 ---
# Config hardware PROVATA, non gestita da raspi-config: due Camera Module 3 (IMX708)
# su cam0/cam1 + UART0 sul 40-pin header per il link seriale allo STM32 (TELEOPPOSE,
# /dev/serial0 -> ttyAMA0 @ 115200; vedi controller/uart/uart_manager.py).
BOOT_CFG=""
[ -f "/boot/firmware/config.txt" ] && BOOT_CFG="/boot/firmware/config.txt"
[ -f "/boot/config.txt" ]          && BOOT_CFG="/boot/config.txt"

if [ -z "$BOOT_CFG" ]; then
    warn "config.txt non trovato — camere/UART da configurare a mano."
elif grep -q "JONNY5 cameras + UART" "${BOOT_CFG}"; then
    ok "Camere + UART già presenti in ${BOOT_CFG} — skip."
else
    cat >> "${BOOT_CFG}" <<'CFG'

# === JONNY5 cameras + UART (added by setup_pi.sh) ===
# [all] forza l'applicazione a tutte le revisioni (niente scoping a [cm4]/[pi5]).
# Due Camera Module 3 (IMX708): cam0 = i2c@88000, cam1 = i2c@80000.
[all]
camera_auto_detect=1
dtoverlay=imx708,cam0
dtoverlay=imx708,cam1
# UART0 sull'header per lo STM32 (la console seriale resta OFF: cmdline = console=tty1).
dtparam=uart0=on
enable_uart=1
dtoverlay=uart0-pi5
CFG
    ok "Camere (IMX708 cam0/cam1) + UART aggiunte a ${BOOT_CFG} (richiede reboot)."
fi

# Guardia: nessuna console di login deve occupare la UART dello STM32.
# (idempotente; su raspi-config recenti tocca solo la console, NON l'hardware UART.)
raspi-config nonint do_serial_cons 1 2>/dev/null || true

# ---------------------------------------------------------------------------
# 4. Hotspot WiFi via NetworkManager
# ---------------------------------------------------------------------------
step "[4/5] Configurazione rete (hotspot WiFi + eth0 ICS/router autoswitch)..."

# Assicura NetworkManager attivo
systemctl enable --now NetworkManager 2>/dev/null || true

# Rimuovi connessione precedente se esiste
nmcli con delete "jonny5-ap" 2>/dev/null || true

# Crea access point
nmcli con add \
    type wifi \
    ifname wlan0 \
    con-name "jonny5-ap" \
    ssid "${WIFI_SSID}" \
    mode ap \
    ipv4.method shared \
    ipv4.addresses "${JONNY5_IP}/24" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "${WIFI_PASSWORD}" \
    connection.autoconnect yes \
    connection.autoconnect-priority 10

ok "Hotspot '${WIFI_SSID}' configurato."

# Attiva subito
nmcli con up "jonny5-ap" 2>/dev/null && ok "Hotspot attivato." || warn "Attivazione posticipata al reboot (wlan0 potrebbe essere in uso)."

# --- eth0: profili ICS-static (.221) + DHCP, commutati dal demone autoswitch ---
# Rimuovi la connessione wired di default creata da NetworkManager (se presente)
nmcli con delete "Wired connection 1" 2>/dev/null || true
# Profilo statico ICS .221 (autoconnect=no: la commutazione la fa j5-net-autoswitch)
nmcli con delete "eth0-ics-static" 2>/dev/null || true
nmcli con add type ethernet ifname eth0 con-name "eth0-ics-static" \
    ipv4.method manual \
    ipv4.addresses "192.168.137.221/24" \
    ipv4.gateway "192.168.137.1" \
    ipv4.dns "192.168.137.1 8.8.8.8" \
    ipv4.route-metric 100 \
    connection.autoconnect no
# Profilo DHCP (default su un router normale, es. casa)
nmcli con delete "eth0-dhcp" 2>/dev/null || true
nmcli con add type ethernet ifname eth0 con-name "eth0-dhcp" \
    ipv4.method auto \
    ipv4.route-metric 50 \
    connection.autoconnect yes
ok "Profili eth0 creati (ics-static .221 + dhcp)."

# Demone autoswitch: al boot forza il profilo giusto in base alla subnet
#   192.168.137.x (ICS Windows) -> .221 ;  192.168.10.x (router casa) -> DHCP
NET_SRC="${SCRIPT_DIR}/raspberry/network"
if [ -f "${NET_SRC}/j5_net_autoswitch.sh" ] && [ -f "${NET_SRC}/j5-net-autoswitch.service" ]; then
    install -m 755 "${NET_SRC}/j5_net_autoswitch.sh" /usr/local/sbin/j5_net_autoswitch.sh
    install -m 644 "${NET_SRC}/j5-net-autoswitch.service" /etc/systemd/system/j5-net-autoswitch.service
    systemctl daemon-reload
    systemctl enable j5-net-autoswitch.service 2>/dev/null \
        && ok "Demone j5-net-autoswitch abilitato (parte al prossimo boot)." \
        || warn "Impossibile abilitare j5-net-autoswitch."
else
    warn "raspberry/network/ non trovato (${NET_SRC}) — demone autoswitch NON installato."
fi

# ---------------------------------------------------------------------------
# 5. Configurazione finale
# ---------------------------------------------------------------------------
step "[5/5] Configurazione finale..."

# NetworkManager: non gestire loopback, abilita WiFi
NMCONF="/etc/NetworkManager/NetworkManager.conf"
if [ -f "$NMCONF" ] && ! grep -q "unmanaged-devices=interface-name:lo" "$NMCONF"; then
    # Assicura che dnsmasq sia abilitato per il captive portal
    if ! grep -q "dns=dnsmasq" "$NMCONF"; then
        sed -i '/\[main\]/a dns=dnsmasq' "$NMCONF"
        ok "dnsmasq abilitato in NetworkManager."
    fi
fi

# Crea directory config_runtime/tls (il deploy.sh la popola coi certificati)
mkdir -p "${REMOTE_DIR}/config_runtime/tls"
chown -R "${JONNY5_USER}:${JONNY5_USER}" "${REMOTE_DIR}"

# Messaggio riepilogo
echo ""
echo -e "${GREEN}=============================================="
echo " Setup completato!"
echo -e "==============================================${NC}"
echo ""
echo " Prossimi passi:"
echo ""
echo "  1. Reboot del Pi (per applicare SPI, camere, UART e hotspot):"
echo "     sudo reboot"
echo "     (post-reboot: rpicam-hello --list-cameras  -> 2x imx708;  ls -l /dev/serial0)"
echo ""
echo "  2. Dal PC, esegui il deploy:"
echo "     ./deploy.sh ${JONNY5_USER}@${JONNY5_IP}"
echo ""
echo "  3. Connettiti all'hotspot '${WIFI_SSID}' dal PC/Quest"
echo "     e visita: https://${JONNY5_IP}/"
echo ""
echo "  Credenziali SSH post-reboot:"
echo "    ssh ${JONNY5_USER}@${JONNY5_IP}"
echo ""
