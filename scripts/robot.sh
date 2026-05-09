#!/bin/bash
# Remote robot management from dev PC via SSH.
#
# Usage: ./robot.sh <command> [-r NAME | -i IP] [-u USER]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/arena_profile.sh"

ROBOT_USER="${ROBOT_USER:-booster}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_PATH="${ROBOT_PATH:-~/spqr/arena}"
SSH_KEY="$HOME/.ssh/id_ed25519"
ROBOT_NAME=""
WIRELESS=""

SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
[ -f "$SSH_KEY" ] && SSH_OPTS="$SSH_OPTS -i $SSH_KEY"

show_help() {
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Options:"
    echo "  -r, --robot NAME   Robot profile name (from scripts/.arena_profiles)"
    echo "  --wireless          Use wireless IP (10.0.19.X)"
    echo "  -i, --ip IP        Direct robot IP (default: $ROBOT_IP)"
    echo "  -u, --user USER    SSH user (default: $ROBOT_USER)"
    echo ""
    echo "Commands:"
    echo "  start     Start arena service"
    echo "  stop      Stop arena service"
    echo "  restart   Restart arena service"
    echo "  status    Show service status"
    echo "  logs      Follow service logs (Ctrl+C to exit)"
    echo "  run       Run arena in foreground (Ctrl+C to stop)"
    echo "  shell     Open interactive shell on robot"
    echo "  ssh       SSH into the robot"
    echo "  install   Install all systemd services + daemons"
    echo "  uninstall Remove all systemd services + daemons"
    echo "  wifi <ssid> <password>  Connect robot to WiFi via nmcli"
    echo "  network   Show robot network status (IPs, interfaces)"
    echo "  profile <set|revert|status> [args]  Manage robot network identity"
}

ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help) show_help; exit 0 ;;
        -r|--robot)  ROBOT_NAME="$2";  shift 2 ;;
        --wireless)  WIRELESS="wireless"; shift ;;
        -i|--ip)     ROBOT_IP="$2";    shift 2 ;;
        -u|--user)   ROBOT_USER="$2";  shift 2 ;;
        *)           ARGS+=("$1");     shift ;;
    esac
done
CMD="${ARGS[0]:-}"
SUBCMD="${ARGS[1]:-}"

if [ -n "$ROBOT_NAME" ]; then
    arena_resolve "$ROBOT_NAME" "$WIRELESS" || exit 1
fi

[ -z "$CMD" ] && { show_help; exit 1; }
arena_ensure_subnet "$ROBOT_IP"
REMOTE="${ROBOT_USER}@${ROBOT_IP}"

case "$CMD" in
    start)   ssh $SSH_OPTS "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh start" ;;
    stop)    ssh $SSH_OPTS "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh stop" ;;
    restart) ssh $SSH_OPTS "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh restart" ;;
    status)  ssh $SSH_OPTS "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh status" ;;
    logs)    ssh $SSH_OPTS "$REMOTE" "journalctl -u arena -f" ;;
    run)     ssh $SSH_OPTS -t "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh run" ;;
    shell)   ssh $SSH_OPTS -t "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh shell" ;;
    ssh)     ssh $SSH_OPTS "$REMOTE" ;;
    install) ssh $SSH_OPTS -t "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh install" ;;
    uninstall) ssh $SSH_OPTS -t "$REMOTE" "cd ${ROBOT_PATH} && ./run.sh uninstall" ;;
    wifi)
        [ -z "${ARGS[1]:-}" ] || [ -z "${ARGS[2]:-}" ] && { echo "Usage: $0 wifi <ssid> <password>"; exit 1; }
        ssid="${ARGS[1]}"; pass="${ARGS[2]}"
        echo "Connecting robot to WiFi: $ssid ..."
        ssh $SSH_OPTS "$REMOTE" "
            sudo nmcli device wifi rescan 2>/dev/null || true
            sudo nmcli device wifi connect '$ssid' password '$pass' 2>/dev/null || {
                echo 'Direct connect failed, trying connection add...'
                sudo nmcli connection add type wifi con-name '$ssid' ssid '$ssid' 2>/dev/null && \
                sudo nmcli connection modify '$ssid' wifi-sec.key-mgmt wpa-psk wifi-sec.psk '$pass' && \
                sudo nmcli connection up '$ssid'
            }
        "
        echo "Done"
        ;;
    network)
        ssh $SSH_OPTS "$REMOTE" "
            echo '=== IP addresses ==='
            ip -4 addr show | grep inet
            echo
            echo '=== Active connections ==='
            nmcli -t -f NAME,TYPE,DEVICE connection show --active 2>/dev/null | grep -E 'wifi|ethernet'
            echo
            echo '=== Identity ==='
            [ -f /etc/robot_identity.yaml ] && cat /etc/robot_identity.yaml || echo 'No identity configured'
        "
        ;;
    profile)
        case "$SUBCMD" in
            set)
                [ -z "$ROBOT_NAME" ] && { echo "Usage: $0 profile set -r NAME [--team N]"; exit 1; }
                "$SCRIPT_DIR/profile.sh" set "$ROBOT_NAME" \
                    "$(awk -v n="$ROBOT_NAME" '$1==n{print $2}' "$PROFILES_FILE" 2>/dev/null)" \
                    -i "$ROBOT_IP"
                WIFI_IP=$(awk -v n="$ROBOT_NAME" -v dt="$DEFAULT_TEAM_NUMBER" \
                    '$1==n{t=($3!=""?$3:dt); print "10.0."t"."$2}' "$PROFILES_FILE" 2>/dev/null)
                [ -n "$WIFI_IP" ] && arena_ensure_subnet "$WIFI_IP"
                ;;
            revert)  "$SCRIPT_DIR/profile.sh" revert "${ROBOT_NAME:-}" -i "$ROBOT_IP" ;;
            status)  "$SCRIPT_DIR/profile.sh" status "${ROBOT_NAME:-}" -i "$ROBOT_IP" ;;
            *) echo "Usage: $0 profile <set|revert|status> -r NAME"; exit 1 ;;
        esac
        ;;
    *)       echo "Unknown command: $CMD"; show_help; exit 1 ;;
esac
