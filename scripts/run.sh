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
    export LD_LIBRARY_PATH="$SCRIPT_DIR/.pixi/aarch64-sysroot/lib64:$SCRIPT_DIR/.pixi/aarch64-sysroot/lib:${LD_LIBRARY_PATH}"
    # Jetson CUDA / TensorRT fallback paths
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda-12.6/targets/aarch64-linux/lib"
    export MODELS_DIR="$SCRIPT_DIR/models"
    export SPQR_SOUNDS_PATH="${SPQR_SOUNDS_PATH:-$SCRIPT_DIR/sounds}"
}

case "$1" in
    shell|bash)
        activate_env
        exec bash
        ;;
    run|runner)
        activate_env
        exec build/aarch64/main --backend booster --task t1-velocity
        ;;
    benchmark)
        shift
        activate_env
        exec build/aarch64/benchmark "$@"
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
        echo "Installing arena services..."
        sudo cp "$SCRIPT_DIR/arena.service"       /etc/systemd/system/arena.service
        sudo cp "$SCRIPT_DIR/arena_start.service" /etc/systemd/system/arena_start.service
        sudo cp "$SCRIPT_DIR/arena_stop.service"  /etc/systemd/system/arena_stop.service
        sudo systemctl daemon-reload
        sudo systemctl enable arena_start
        sudo systemctl enable arena_stop
        sudo systemctl restart arena_start
        sudo systemctl restart arena_stop
        echo "Daemons installed and started (arena requires joystick activation)."
        ;;
    uninstall)
        echo "Uninstalling arena services..."
        sudo systemctl stop "$SERVICE_NAME" 2>/dev/null || true
        sudo systemctl disable "$SERVICE_NAME" 2>/dev/null || true
        sudo rm -f /etc/systemd/system/arena.service
        sudo systemctl stop arena_start 2>/dev/null || true
        sudo systemctl stop arena_stop 2>/dev/null || true
        sudo systemctl disable arena_start 2>/dev/null || true
        sudo systemctl disable arena_stop 2>/dev/null || true
        sudo rm -f /etc/systemd/system/arena_start.service
        sudo rm -f /etc/systemd/system/arena_stop.service
        sudo systemctl daemon-reload
        echo "Services uninstalled"
        ;;
    daemon-start)
        sudo systemctl restart arena_start
        sudo systemctl restart arena_stop
        echo "Start/stop daemons restarted"
        ;;
    daemon-stop)
        sudo systemctl stop arena_start arena_stop
        echo "Start/stop daemons stopped"
        ;;
    daemon-logs)
        journalctl -u arena-start -u arena-stop -f
        ;;
    *)
        echo "Usage: $0 <command>"
        echo ""
        echo "  shell    Start interactive shell with environment activated"
        echo "  run       Run arena in foreground"
        echo "  benchmark Run benchmark (pass args after)"
        echo "  gdb       Run arena under gdb"
        echo ""
        echo "  start    Start arena service"
        echo "  stop     Stop arena service"
        echo "  restart  Restart arena service"
        echo "  status   Show arena service status"
        echo "  logs     Follow arena service logs"
        echo ""
        echo "  daemon-start  Restart start/stop daemons"
        echo "  daemon-stop   Stop start/stop daemons"
        echo "  daemon-logs   Follow daemon logs"
        echo ""
        echo "  install  Install all systemd services"
        echo "  uninstall Remove all systemd services"
        exit 1
        ;;
esac
