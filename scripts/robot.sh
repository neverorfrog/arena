#!/bin/bash
# Remote robot management from dev PC via SSH.
#
# Usage: ./robot.sh <command> [-i IP] [-u USER]

set -e

ROBOT_USER="${ROBOT_USER:-booster}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_PATH="${ROBOT_PATH:-~/spqr/arena}"
SSH_KEY="$HOME/.ssh/id_ed25519"

SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
[ -f "$SSH_KEY" ] && SSH_OPTS="$SSH_OPTS -i $SSH_KEY"

show_help() {
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Options:"
    echo "  -i, --ip IP       Robot IP (default: $ROBOT_IP)"
    echo "  -u, --user USER   SSH user (default: $ROBOT_USER)"
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
}

CMD=""
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help) show_help; exit 0 ;;
        -i|--ip)   ROBOT_IP="$2"; shift 2 ;;
        -u|--user) ROBOT_USER="$2"; shift 2 ;;
        *)
            if [ -z "$CMD" ]; then CMD="$1"; shift
            else echo "Unknown argument: $1"; exit 1
            fi ;;
    esac
done

[ -z "$CMD" ] && { show_help; exit 1; }
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
    *)       echo "Unknown command: $CMD"; show_help; exit 1 ;;
esac
