#!/bin/bash
# Deploy arena to the robot: cross-compile, transfer binary, restart service.
#
# Options:
#   -r, --robot NAME    Robot profile name (from scripts/.arena_profiles)
#   --wireless           Use wireless IP (10.0.19.X) instead of wired (192.168.19.X)
#   -i, --ip IP         Direct robot IP (overrides -r)
#   -u, --user USER     SSH username    (default: booster)
#   --path PATH         Install path on robot (default: ~/spqr/arena)
#   --skip-build        Skip cross-compilation (reuse existing binary)
#   --skip-restart      Skip service restart after deploy (default)
#   --restart           Restart arena service after deploy

set -e

GREEN='\033[0;32m' BLUE='\033[0;34m' YELLOW='\033[1;33m' RED='\033[0;31m' NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/arena_profile.sh"

ROBOT_USER="${ROBOT_USER:-booster}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_PATH="${ROBOT_PATH:-~/spqr/arena}"
ROBOT_PASSWORD="${ROBOT_PASSWORD:-123456}"
SKIP_BUILD=false
SKIP_RESTART=true
ROBOT_NAME=""
WIRELESS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--robot)     ROBOT_NAME="$2";  shift 2 ;;
        --wireless)     WIRELESS="wireless"; shift ;;
        -i|--ip)        ROBOT_IP="$2";    shift 2 ;;
        -u|--user)      ROBOT_USER="$2";  shift 2 ;;
        --path)         ROBOT_PATH="$2";  shift 2 ;;
        --skip-build)   SKIP_BUILD=true;  shift ;;
        --skip-restart) SKIP_RESTART=true; shift ;;
        --restart)      SKIP_RESTART=false; shift ;;
        -h|--help)      head -n 12 "$0" | tail -n +2 | sed 's/^# \?//'; exit 0 ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; exit 1 ;;
    esac
done

if [ -n "$ROBOT_NAME" ]; then
    arena_resolve "$ROBOT_NAME" "$WIRELESS" || exit 1
fi

ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
arena_ensure_subnet "$ROBOT_IP"
SSH_KEY="$HOME/.ssh/id_ed25519"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -i $SSH_KEY"
ROBOT="${ROBOT_USER}@${ROBOT_IP}"
RSUDO="echo '$ROBOT_PASSWORD' | sudo -S -p ''"

echo -e "${BLUE}=== Arena Deploy → ${ROBOT}:${ROBOT_PATH} ===${NC}"

# ── Step 1: Build ────────────────────────────────────────────────────────────
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${GREEN}[1/3] Cross-compiling for aarch64...${NC}"
    cd "$ROOT"
    pixi run compile-aarch64
    echo -e "  ${GREEN}✓${NC} Build complete"
else
    echo -e "${YELLOW}[1/3] Skipping build${NC}"
fi
echo

# ── Step 2: Transfer ─────────────────────────────────────────────────────────
echo -e "${GREEN}[2/3] Transferring to robot...${NC}"

BUILD_DIR="$ROOT/build/aarch64"
if [ ! -d "$BUILD_DIR" ]; then
    echo -e "  ${RED}Build directory not found at $BUILD_DIR${NC}"
    exit 1
fi

ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}/build/aarch64 ${ROBOT_PATH}/models ${ROBOT_PATH}/sounds"

echo -e "  ${BLUE}build...${NC}"
rsync -aqz -e "ssh $SSH_OPTS" --delete \
    --exclude="CMakeFiles" \
    --exclude="cmake_install.cmake" \
    --exclude="Makefile" \
    --exclude="CTestTestfile.cmake" \
    "$BUILD_DIR/" "$ROBOT:${ROBOT_PATH}/build/aarch64/"

echo -e "  ${BLUE}models...${NC}"
rsync -aqz -e "ssh $SSH_OPTS" \
    "$ROOT/models/" "$ROBOT:${ROBOT_PATH}/models/"

echo -e "  ${BLUE}sounds...${NC}"
rsync -aqz -e "ssh $SSH_OPTS" \
    "$ROOT/sounds/" "$ROBOT:${ROBOT_PATH}/sounds/"

for f in run.sh arena.service arena_start.service arena_stop.service dds_debug.service; do
    scp -q $SSH_OPTS "$SCRIPT_DIR/$f" "$ROBOT:${ROBOT_PATH}/$f"
done
ssh $SSH_OPTS "$ROBOT" "chmod +x ${ROBOT_PATH}/run.sh"

# Always install/refresh the service unit files so they stay in sync.
echo -e "  ${BLUE}systemd units...${NC}"
ssh $SSH_OPTS "$ROBOT" "\
    $RSUDO cp ${ROBOT_PATH}/arena.service       /etc/systemd/system/arena.service && \
    $RSUDO cp ${ROBOT_PATH}/arena_start.service  /etc/systemd/system/arena_start.service && \
    $RSUDO cp ${ROBOT_PATH}/arena_stop.service   /etc/systemd/system/arena_stop.service && \
    $RSUDO cp ${ROBOT_PATH}/dds_debug.service    /etc/systemd/system/dds-debug.service && \
    $RSUDO systemctl daemon-reload"

echo -e "  ${GREEN}✓${NC} Files transferred"
echo

# ── Step 3: Restart service ──────────────────────────────────────────────────
if [ "$SKIP_RESTART" = false ]; then
    echo -e "${GREEN}[3/3] Restarting arena service...${NC}"
    if ssh $SSH_OPTS "$ROBOT" "systemctl is-active --quiet arena" 2>/dev/null; then
        ssh $SSH_OPTS "$ROBOT" "$RSUDO systemctl restart arena"
        echo -e "  ${GREEN}✓${NC} Service restarted"
    else
        ssh $SSH_OPTS "$ROBOT" "$RSUDO systemctl start arena"
        echo -e "  ${GREEN}✓${NC} Service started"
    fi
else
    echo -e "  ${YELLOW}Skipping restart${NC}"
fi

echo -e "\n${GREEN}Deploy complete!${NC}"
