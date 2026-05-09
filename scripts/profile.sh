#!/bin/bash
# Robot profile manager — registry and remote network configuration.
# Always runs on the dev PC; talks to the robot over SSH for network commands.
#
# Usage:
#   profile.sh set <name> <number> [--team N] [-i IP]   # register + configure robot
#   profile.sh revert <name> [-i IP]                     # restore robot's previous config
#   profile.sh status <name> [-i IP]                     # show robot's live identity
#   profile.sh list                                       # list all profiles
#   profile.sh show <name>                                # show profile IPs
#   profile.sh remove <name>                              # remove from registry

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILES_FILE="$SCRIPT_DIR/.arena_profiles"
DEFAULT_TEAM=19
ROBOT_USER="${ROBOT_USER:-booster}"
SSH_KEY="$HOME/.ssh/id_ed25519"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=5"
[ -f "$SSH_KEY" ] && SSH_OPTS="$SSH_OPTS -i $SSH_KEY"

die() { echo "ERROR: $*" >&2; exit 1; }

_read_profile() {
    local name="$1"
    [ -f "$PROFILES_FILE" ] || die "No profiles file. Run: profile.sh set <name> <number>"
    awk -v n="$name" -v dt="$DEFAULT_TEAM" '$1==n {print $2, ($3!=""?$3:dt)}' "$PROFILES_FILE"
}

_resolve_ip() {
    local name="$1"
    local row; row=$(_read_profile "$name") || exit 1
    local num team
    num=$(awk '{print $1}' <<< "$row")
    team=$(awk '{print $2}' <<< "$row")
    echo "192.168.${team}.${num}"
}

# ── Commands ──────────────────────────────────────────────────────────────────

cmd_set() {
    local name="${1:-}" number="${2:-}"
    [ -z "$name" ]   && die "Usage: profile.sh set <name> <number> [--team N] [-i IP]"
    [ -z "$number" ] && die "Usage: profile.sh set <name> <number> [--team N] [-i IP]"
    shift 2

    local team="$DEFAULT_TEAM"
    local connect_ip="192.168.10.102"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --team)    team="$2";       shift 2 ;;
            -i|--ip)   connect_ip="$2"; shift 2 ;;
            *) die "Unknown option: $1" ;;
        esac
    done

    local wired_ip="192.168.${team}.${number}"
    local wifi_ip="10.0.${team}.${number}"
    local wifi_gw="10.0.${team}.1"
    local remote="${ROBOT_USER}@${connect_ip}"

    # Update local registry
    if grep -q "^${name} " "$PROFILES_FILE" 2>/dev/null; then
        sed -i "s/^${name} .*/${name} ${number} ${team}/" "$PROFILES_FILE"
    else
        echo "${name} ${number} ${team}" >> "$PROFILES_FILE"
    fi
    echo "Registry: ${name} → wired ${wired_ip}, wifi ${wifi_ip}"

    # Configure robot over SSH
    echo "Connecting to ${connect_ip}..."
    ssh $SSH_OPTS "$remote" bash << EOF
set -e

# ── Wired: add secondary IP (keep factory 192.168.10.102) ─────────────────────
sudo nmcli connection modify eth0 ipv4.method manual

CURRENT=\$(sudo nmcli -g ipv4.addresses connection show eth0 2>/dev/null || echo "")
if ! echo "\$CURRENT" | grep -qF "192.168.10.102"; then
    sudo nmcli connection modify eth0 +ipv4.addresses "192.168.10.102/24"
fi

# Remove any previously set team IP stored in identity file
OLD_IP=\$(awk -F'"' '/^wired_ip/{print \$2}' /etc/robot_identity.yaml 2>/dev/null || echo "")
if [ -n "\$OLD_IP" ] && [ "\$OLD_IP" != "192.168.10.102" ] && [ "\$OLD_IP" != "${wired_ip}" ]; then
    sudo nmcli connection modify eth0 -ipv4.addresses "\${OLD_IP}/24" 2>/dev/null || true
fi

if ! echo "\$CURRENT" | grep -qF "${wired_ip}"; then
    sudo nmcli connection modify eth0 +ipv4.addresses "${wired_ip}/24"
fi
sudo nmcli connection up eth0 2>/dev/null || true

# ── WiFi: replace with static IP (save prev config for revert) ────────────────
WIFI_IFACE=\$(sudo nmcli -t -f DEVICE,TYPE,STATE device 2>/dev/null | awk -F: '\$2=="wifi" && \$3=="connected" {print \$1; exit}')
WIFI_CONN=\$([ -n "\$WIFI_IFACE" ] && sudo nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null | awk -F: -v d="\$WIFI_IFACE" '\$2==d {print \$1; exit}' || echo "")
WIFI_PREV_METHOD="auto"
WIFI_PREV_ADDRS=""
WIFI_PREV_GW=""

if [ -n "\$WIFI_CONN" ]; then
    WIFI_PREV_METHOD=\$(sudo nmcli -g ipv4.method    connection show "\$WIFI_CONN" 2>/dev/null || echo auto)
    WIFI_PREV_ADDRS=\$(sudo nmcli -g ipv4.addresses  connection show "\$WIFI_CONN" 2>/dev/null || echo "")
    WIFI_PREV_GW=\$(sudo nmcli    -g ipv4.gateway    connection show "\$WIFI_CONN" 2>/dev/null || echo "")

    sudo nmcli connection modify "\$WIFI_CONN" \
        ipv4.method    manual \
        ipv4.addresses "${wifi_ip}/24" \
        ipv4.gateway   "${wifi_gw}" \
        ipv4.dns       "8.8.8.8"
    sudo nmcli connection up "\$WIFI_CONN" 2>/dev/null || true
fi

# ── Identity file ──────────────────────────────────────────────────────────────
printf 'name: "%s"\nnumber: %s\nteam: %s\nwired_ip: "%s"\nwifi_ip: "%s"\nwifi_conn: "%s"\nwifi_prev_method: "%s"\nwifi_prev_addrs: "%s"\nwifi_prev_gateway: "%s"\n' \
    "${name}" "${number}" "${team}" "${wired_ip}" "${wifi_ip}" \
    "\$WIFI_CONN" "\$WIFI_PREV_METHOD" "\$WIFI_PREV_ADDRS" "\$WIFI_PREV_GW" \
    | sudo tee /etc/robot_identity.yaml > /dev/null

echo "  wired:    ${wired_ip}"
[ -n "\$WIFI_CONN" ] && echo "  wifi:     ${wifi_ip} (\$WIFI_CONN)" || echo "  wifi:     no active connection found, skipped"
EOF

    echo "Done. Reachable at ${ROBOT_USER}@${wired_ip}"
}

cmd_revert() {
    local name="${1:-}"; [ -z "$name" ] && die "Usage: profile.sh revert <name> [-i IP]"
    shift

    local connect_ip; connect_ip=$(_resolve_ip "$name")
    while [[ $# -gt 0 ]]; do
        case $1 in
            -i|--ip) connect_ip="$2"; shift 2 ;;
            *) die "Unknown option: $1" ;;
        esac
    done

    local remote="${ROBOT_USER}@${connect_ip}"
    echo "Reverting robot at ${connect_ip}..."
    ssh $SSH_OPTS "$remote" bash << 'EOF'
set -e
ID=/etc/robot_identity.yaml
[ -f "$ID" ] || { echo "No identity file — nothing to revert"; exit 1; }

WIRED_IP=$(awk    -F'"' '/^wired_ip/{print $2}'        "$ID")
WIFI_CONN=$(awk   -F'"' '/^wifi_conn/{print $2}'        "$ID")
PREV_METHOD=$(awk -F'"' '/^wifi_prev_method/{print $2}' "$ID")
PREV_ADDRS=$(awk  -F'"' '/^wifi_prev_addrs/{print $2}'  "$ID")
PREV_GW=$(awk     -F'"' '/^wifi_prev_gateway/{print $2}' "$ID")

# Remove team IP from eth0
if [ -n "$WIRED_IP" ] && [ "$WIRED_IP" != "192.168.10.102" ]; then
    sudo nmcli connection modify eth0 -ipv4.addresses "${WIRED_IP}/24" 2>/dev/null || true
    sudo nmcli connection up eth0 2>/dev/null || true
fi

# Restore WiFi
if [ -n "$WIFI_CONN" ]; then
    sudo nmcli connection modify "$WIFI_CONN" \
        ipv4.method   "${PREV_METHOD:-auto}" \
        ipv4.addresses "${PREV_ADDRS}" \
        ipv4.gateway   "${PREV_GW}" \
        ipv4.dns       ""
    sudo nmcli connection up "$WIFI_CONN" 2>/dev/null || true
fi

sudo rm -f "$ID"
echo "Reverted. Identity file removed."
EOF
}

cmd_status() {
    local name="${1:-}"; [ -z "$name" ] && die "Usage: profile.sh status <name> [-i IP]"
    shift

    local connect_ip; connect_ip=$(_resolve_ip "$name")
    while [[ $# -gt 0 ]]; do
        case $1 in
            -i|--ip) connect_ip="$2"; shift 2 ;;
            *) die "Unknown option: $1" ;;
        esac
    done

    ssh $SSH_OPTS "${ROBOT_USER}@${connect_ip}" \
        "[ -f /etc/robot_identity.yaml ] && cat /etc/robot_identity.yaml || echo 'No identity configured'"
}

cmd_list() {
    if [ ! -f "$PROFILES_FILE" ] || [ ! -s "$PROFILES_FILE" ]; then
        echo "No profiles found."; return
    fi
    echo "Profiles (default team: ${DEFAULT_TEAM}):"
    while read -r name num team _; do
        [ -z "$name" ] && continue
        team="${team:-$DEFAULT_TEAM}"
        printf "  %-12s  #%-3s  team %-3s  wired: 192.168.%s.%-3s  wifi: 10.0.%s.%-3s\n" \
            "$name" "$num" "$team" "$team" "$num" "$team" "$num"
    done < "$PROFILES_FILE"
}

cmd_show() {
    local name="${1:-}"; [ -z "$name" ] && die "Usage: profile.sh show <name>"
    local row; row=$(_read_profile "$name")
    local num team
    num=$(awk '{print $1}' <<< "$row")
    team=$(awk '{print $2}' <<< "$row")
    echo "Profile: $name"
    echo "  number:    $num"
    echo "  team:      $team"
    echo "  wired:     192.168.${team}.${num}"
    echo "  wireless:  10.0.${team}.${num}"
}

cmd_remove() {
    local name="${1:-}"; [ -z "$name" ] && die "Usage: profile.sh remove <name>"
    sed -i "/^${name} /d" "$PROFILES_FILE"
    echo "Profile '${name}' removed"
}

# ── Dispatch ──────────────────────────────────────────────────────────────────

CMD="${1:-}"; [ -n "$CMD" ] && shift || true

case "$CMD" in
    set)    cmd_set    "$@" ;;
    revert) cmd_revert "$@" ;;
    status) cmd_status "$@" ;;
    list)   cmd_list ;;
    show)   cmd_show   "$@" ;;
    remove) cmd_remove "$@" ;;
    -h|--help|"")
        echo "Usage: $0 <command> [args]"
        echo ""
        echo "  set <name> <number> [--team N] [-i IP]   Register and configure robot"
        echo "  revert <name> [-i IP]                     Restore previous network config"
        echo "  status <name> [-i IP]                     Show robot's live identity"
        echo "  list                                       List all profiles"
        echo "  show <name>                                Show profile IPs"
        echo "  remove <name>                              Delete profile"
        exit 0 ;;
    *) die "Unknown command: $CMD. Run '$0 --help' for usage." ;;
esac
