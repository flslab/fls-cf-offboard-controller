#!/bin/bash
# =============================================================================
# network_switch.sh — FLS Swarm Network Mode Switcher (runs on drone / Pi)
#
# Usage:
#   network_switch.sh adhoc     <ip> <ssid> <channel> [interface]
#   network_switch.sh wifi      [interface]
#   network_switch.sh bluetooth <controller_mac> <ip> [bt_interface]
#
# Examples:
#   network_switch.sh adhoc 10.0.0.1 fls-adhoc 6 wlan0
#   network_switch.sh wifi wlan0
#   network_switch.sh bluetooth AA:BB:CC:DD:EE:FF 10.0.0.1 hci0
# =============================================================================

set -euo pipefail

MODE="${1:-}"
LOG_TAG="[net-switch]"

# ── helpers ──────────────────────────────────────────────────────────────────
die()  { echo "$LOG_TAG ERROR: $*" >&2; exit 1; }
info() { echo "$LOG_TAG $*"; }

require_root() {
    if [[ $EUID -ne 0 ]]; then
        exec sudo "$0" "$@"
    fi
}

nmcli_delete_if_exists() {
    local con_name="$1"
    nmcli connection delete "$con_name" 2>/dev/null && info "Removed old connection '$con_name'." || true
}

# ── ADHOC ─────────────────────────────────────────────────────────────────────
switch_adhoc() {
    local ip="${1:-}"   ssid="${2:-fls-adhoc}"
    local channel="${3:-6}"   iface="${4:-wlan0}"
    local con_name="fls-adhoc"

    [[ -z "$ip" ]] && die "adhoc requires <ip> argument"

    info "Switching to Wi-Fi ad-hoc (SSID=$ssid  IP=$ip/24  ch=$channel  iface=$iface)"

    # Make sure wlan0 is not blocked
    rfkill unblock wifi 2>/dev/null || true

    # Remove stale connection (ignore errors)
    nmcli_delete_if_exists "$con_name"

    # Create ad-hoc (IBSS) connection with a static IP
    nmcli connection add \
        type wifi \
        con-name  "$con_name" \
        ifname    "$iface" \
        ssid      "$ssid" \
        -- \
        wifi.mode             adhoc \
        wifi.channel          "$channel" \
        ipv4.method           manual \
        ipv4.addresses        "$ip/24" \
        ipv6.method           disabled \
        connection.autoconnect no

    nmcli connection up "$con_name"

    info "Ad-hoc active.  IP: $ip  SSID: $ssid"
}

# ── WIFI (managed) ────────────────────────────────────────────────────────────
switch_wifi() {
    local iface="${1:-wlan0}"

    info "Returning to managed Wi-Fi (iface=$iface)..."

    # Take down any ad-hoc / BT connections we created
    nmcli connection down "fls-adhoc"  2>/dev/null || true
    nmcli connection down "fls-bt-pan" 2>/dev/null || true

    # Let NetworkManager auto-connect to the saved infrastructure network
    nmcli device connect "$iface" 2>/dev/null || \
        nmcli device set    "$iface" autoconnect yes

    info "Managed Wi-Fi activated on $iface."
}

# ── BLUETOOTH PAN ─────────────────────────────────────────────────────────────
switch_bluetooth() {
    local ctrl_mac="${1:-}"   ip="${2:-}"   bt_iface="${3:-hci0}"
    local con_name="fls-bt-pan"

    [[ -z "$ctrl_mac" ]] && die "bluetooth requires <controller_mac>"
    [[ -z "$ip"       ]] && die "bluetooth requires <ip>"

    info "Connecting to Bluetooth PAN  ctrl=$ctrl_mac  ip=$ip/24"

    # Ensure BT hardware is on
    rfkill unblock bluetooth 2>/dev/null || true
    systemctl --quiet is-active bluetooth || systemctl start bluetooth
    sleep 1

    bluetoothctl power on                 >/dev/null
    bluetoothctl agent on                 >/dev/null 2>&1 || true
    bluetoothctl default-agent            >/dev/null 2>&1 || true

    # Scan briefly to discover the controller if not already known
    if ! bluetoothctl devices | grep -qi "$ctrl_mac"; then
        info "Scanning for controller ($ctrl_mac) ..."
        bluetoothctl scan on  >/dev/null &
        SCAN_PID=$!
        sleep 8
        kill "$SCAN_PID" 2>/dev/null || true
        bluetoothctl scan off >/dev/null 2>&1 || true
    fi

    # Pair + trust (safe to re-run)
    bluetoothctl pair  "$ctrl_mac" >/dev/null 2>&1 || info "Pair skipped (may already be paired)"
    bluetoothctl trust "$ctrl_mac" >/dev/null

    # Remove stale NM connection
    nmcli_delete_if_exists "$con_name"

    # Create BT PAN (PANU) connection
    nmcli connection add \
        type          bluetooth \
        con-name      "$con_name" \
        bluetooth.bdaddr "$ctrl_mac" \
        bluetooth.type   panu \
        ipv4.method   manual \
        ipv4.addresses "$ip/24" \
        ipv6.method   disabled \
        connection.autoconnect no

    nmcli connection up "$con_name"
    info "Bluetooth PAN active.  IP: $ip"
}

# ── DISPATCH ──────────────────────────────────────────────────────────────────
case "$MODE" in
    adhoc)
        require_root "$@"
        switch_adhoc "${2:-}" "${3:-fls-adhoc}" "${4:-6}" "${5:-wlan0}"
        ;;
    wifi)
        require_root "$@"
        switch_wifi "${2:-wlan0}"
        ;;
    bluetooth)
        require_root "$@"
        switch_bluetooth "${2:-}" "${3:-}" "${4:-hci0}"
        ;;
    "")
        echo "Usage: $0 {adhoc|wifi|bluetooth} [args...]"
        echo ""
        echo "  adhoc     <ip> [ssid=fls-adhoc] [channel=6] [iface=wlan0]"
        echo "  wifi      [iface=wlan0]"
        echo "  bluetooth <controller_mac> <ip> [bt_iface=hci0]"
        exit 1
        ;;
    *)
        die "Unknown mode '$MODE'. Choose: adhoc | wifi | bluetooth"
        ;;
esac
