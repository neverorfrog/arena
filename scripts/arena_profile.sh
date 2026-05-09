# Source this to resolve robot name -> IP from scripts/.arena_profiles
# Sets: ROBOT_IP, ROBOT_USER
# Profile format: NAME NUMBER [TEAM]  (TEAM defaults to 19 if omitted)

DEFAULT_TEAM_NUMBER=19
if [ -z "$SCRIPT_DIR" ]; then
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi
PROFILES_FILE="$SCRIPT_DIR/.arena_profiles"

arena_resolve() {
    local name="$1"
    local wireless="$2"
    ROBOT_USER="${ROBOT_USER:-booster}"

    if [ ! -f "$PROFILES_FILE" ]; then
        echo "ERROR: $PROFILES_FILE not found. Run: ./scripts/profile.sh set <name> <number>" >&2
        return 1
    fi

    local num team
    read -r num team <<< "$(awk -v n="$name" -v dt="$DEFAULT_TEAM_NUMBER" \
        '$1==n {print $2, ($3!=""?$3:dt)}' "$PROFILES_FILE")"
    [ -z "$num" ] && { echo "ERROR: robot '$name' not found in $PROFILES_FILE" >&2; return 1; }

    if [ "$wireless" = "wireless" ]; then
        ROBOT_IP="10.0.${team}.${num}"
    else
        ROBOT_IP="192.168.${team}.${num}"
    fi
}

arena_ensure_subnet() {
    local target_ip="$1"
    local subnet_prefix="${target_ip%.*}"
    local my_ip="${subnet_prefix}.10/24"

    # Already on this subnet?
    ip -4 addr show | grep -q "inet ${subnet_prefix}\." && return 0

    # Find an UP interface: prefer wired (en*), fall back to WiFi (wl*)
    local iface
    iface=$(ip -o link show 2>/dev/null | grep -oP '(en|wl)[a-z0-9]+' | while read -r if; do
        ip link show "$if" 2>/dev/null | grep -q 'state UP' && echo "$if" && break
    done)
    [ -z "$iface" ] && return 0

    # Already have this IP on this interface?
    ip -4 addr show "$iface" 2>/dev/null | grep -q "$my_ip" && return 0

    # Try without sudo, then with sudo (prompts for password if TTY)
    if ip addr add "$my_ip" dev "$iface" 2>/dev/null; then
        echo "  Added secondary IP $my_ip on $iface"
    elif sudo -v 2>/dev/null && sudo -n ip addr add "$my_ip" dev "$iface" 2>/dev/null; then
        echo "  Added secondary IP $my_ip on $iface"
    else
        echo "  Run: sudo ip addr add $my_ip dev $iface"
    fi
}
