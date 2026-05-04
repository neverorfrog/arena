#!/bin/bash
# Run arena on the robot. Lives at $ROBOT_PATH/run.sh after setup_robot.sh.
# Usage: ./run.sh <command>

SERVICE_NAME="arena"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

activate_env() {
    if [ ! -d "$SCRIPT_DIR/.pixi/aarch64-sysroot/lib" ]; then
        echo "Error: Sysroot not found — run setup_robot.sh first"
        exit 1
    fi
    export LD_LIBRARY_PATH="$SCRIPT_DIR/.pixi/aarch64-sysroot/lib:${LD_LIBRARY_PATH}"
    # Jetson CUDA / TensorRT fallback paths
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda-12.6/targets/aarch64-linux/lib"
    export MODELS_DIR="$SCRIPT_DIR/models"
}

case "$1" in
    shell|bash)
        activate_env
        exec bash
        ;;
    run|runner)
        activate_env
        exec build/aarch64/main --backend booster --task t1-velocity-flat
        ;;
    gdb)
        activate_env
        exec gdb build/aarch64/main
        ;;
    start)
        if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
            echo "Service is already running"
        else
            sudo systemctl start "$SERVICE_NAME"
            echo "Started. Use './run.sh logs' to follow logs"
        fi
        ;;
    stop)
        if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
            sudo systemctl stop "$SERVICE_NAME"
            echo "Stopped"
        else
            echo "Service is not running"
        fi
        ;;
    restart)
        sudo systemctl restart "$SERVICE_NAME"
        echo "Restarted"
        ;;
    status)
        systemctl status "$SERVICE_NAME" --no-pager
        ;;
    logs)
        journalctl -u "$SERVICE_NAME" -f
        ;;
    install)
        echo "Installing arena service..."
        sudo cp "$SCRIPT_DIR/arena.service" /etc/systemd/system/arena.service
        sudo systemctl daemon-reload
        echo "Service installed (not auto-started — use './run.sh start')"
        ;;
    uninstall)
        echo "Uninstalling arena service..."
        sudo systemctl stop "$SERVICE_NAME" 2>/dev/null || true
        sudo systemctl disable "$SERVICE_NAME" 2>/dev/null || true
        sudo rm -f /etc/systemd/system/arena.service
        sudo systemctl daemon-reload
        echo "Service uninstalled"
        ;;
    *)
        echo "Usage: $0 <command>"
        echo ""
        echo "  shell    Start interactive shell with environment activated"
        echo "  run      Run arena in foreground"
        echo "  gdb      Run arena under gdb"
        echo ""
        echo "  start    Start systemd service"
        echo "  stop     Stop systemd service"
        echo "  restart  Restart systemd service"
        echo "  status   Show service status"
        echo "  logs     Follow service logs"
        echo ""
        echo "  install  Install systemd service"
        echo "  uninstall Remove systemd service"
        exit 1
        ;;
esac
