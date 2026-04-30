#!/bin/bash
# Run gladius on the robot. Lives at $ROBOT_PATH/run.sh after deploy.
# Usage: ./run.sh <command>

SERVICE_NAME="arena"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

activate_env() {
    if [ ! -f "$SCRIPT_DIR/activate.sh" ]; then
        echo "Error: environment not found — run deploy.sh first"
        exit 1
    fi
    source "$SCRIPT_DIR/activate.sh"
    export LD_LIBRARY_PATH="$SCRIPT_DIR/env/lib:${LD_LIBRARY_PATH}"
    export MODELS_DIR="$SCRIPT_DIR/models"
}

case "$1" in
    shell)
        activate_env
        exec bash
        ;;
    run)
        activate_env
        exec main
        ;;
    gdb)
        activate_env
        exec gdb main
        ;;
    start)
        sudo systemctl start "$SERVICE_NAME"
        ;;
    stop)
        sudo systemctl stop "$SERVICE_NAME"
        ;;
    restart)
        sudo systemctl restart "$SERVICE_NAME"
        ;;
    status)
        systemctl status "$SERVICE_NAME" --no-pager
        ;;
    logs)
        journalctl -u "$SERVICE_NAME" -f
        ;;
    *)
        echo "Usage: $0 <command>"
        echo ""
        echo "  shell    Start shell with environment activated"
        echo "  run      Run main (foreground)"
        echo "  gdb      Run main under gdb"
        echo ""
        echo "  start    Start systemd service"
        echo "  stop     Stop systemd service"
        echo "  restart  Restart systemd service"
        echo "  status   Show service status"
        echo "  logs     Follow service logs"
        exit 1
        ;;
esac
