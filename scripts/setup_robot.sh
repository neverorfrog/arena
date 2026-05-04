#!/bin/bash
# One-time robot setup: transfer aarch64-sysroot, models, and install services.
#
# Options:
#   -i, --ip IP         Robot IP address (default: 192.168.10.102)
#   -u, --user USER     SSH username    (default: booster)
#   --path PATH         Install path on robot (default: ~/spqr/arena)

set -e

GREEN='\033[0;32m' BLUE='\033[0;34m' YELLOW='\033[1;33m' RED='\033[0;31m' NC='\033[0m'

ROBOT_USER="${ROBOT_USER:-booster}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_PATH="${ROBOT_PATH:-~/spqr/arena}"

while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--ip)   ROBOT_IP="$2";   shift 2 ;;
        -u|--user) ROBOT_USER="$2"; shift 2 ;;
        --path)    ROBOT_PATH="$2"; shift 2 ;;
        -h|--help) head -n 7 "$0" | tail -n +2 | sed 's/^# \?//'; exit 0 ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; exit 1 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
SSH_KEY="$HOME/.ssh/id_ed25519"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -i $SSH_KEY"
ROBOT="${ROBOT_USER}@${ROBOT_IP}"

SYSROOT="$ROOT/.pixi/aarch64-sysroot"

echo -e "${BLUE}=== Arena Robot Setup → ${ROBOT}:${ROBOT_PATH} ===${NC}"

# ── SSH key ──────────────────────────────────────────────────────────────────
setup_ssh() {
    if [ ! -f "$SSH_KEY" ]; then
        ssh-keygen -t ed25519 -f "$SSH_KEY" -N "" -C "arena@$(hostname)" >/dev/null 2>&1
        echo -e "  ${GREEN}✓${NC} SSH key generated"
    fi
    if ssh $SSH_OPTS -o PasswordAuthentication=no -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT" exit 2>/dev/null; then
        return
    fi
    echo -e "  Copying SSH key to robot (password required once)..."
    ssh-copy-id -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -i "$SSH_KEY" "$ROBOT"
}

# ── Step 1: Ensure sysroot exists ────────────────────────────────────────────
echo -e "${GREEN}[1/4] Checking aarch64 sysroot...${NC}"
if [ ! -d "$SYSROOT/lib" ]; then
    echo -e "  ${RED}aarch64 sysroot not found. Run 'pixi run setup' first.${NC}"
    exit 1
fi
echo -e "  ${GREEN}✓${NC} Sysroot ready"
echo

# ── Step 2: Transfer sysroot ─────────────────────────────────────────────────
echo -e "${GREEN}[2/4] Transferring sysroot to robot...${NC}"
setup_ssh

ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}"

# Exclude dev-only directories to save transfer time
echo -e "  Transferring sysroot (libs only)..."
rsync -az -e "ssh $SSH_OPTS" --delete \
    --exclude="include" \
    --exclude="share" \
    --exclude="targets" \
    "$SYSROOT/" "$ROBOT:${ROBOT_PATH}/.pixi/aarch64-sysroot/"
echo -e "  ${GREEN}✓${NC} Sysroot transferred"
echo

# ── Step 3: Transfer models and runtime files ────────────────────────────────
echo -e "${GREEN}[3/4] Transferring models and runtime files...${NC}"

ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}/models"

echo -e "  Syncing models..."
rsync -az -e "ssh $SSH_OPTS" \
    "$ROOT/external/colosseum/models/" "$ROBOT:${ROBOT_PATH}/models/"

scp $SSH_OPTS "$SCRIPT_DIR/run.sh"       "$ROBOT:${ROBOT_PATH}/run.sh"
scp $SSH_OPTS "$SCRIPT_DIR/arena.service" "$ROBOT:${ROBOT_PATH}/arena.service"
ssh $SSH_OPTS "$ROBOT" "chmod +x ${ROBOT_PATH}/run.sh"

echo -e "  ${GREEN}✓${NC} Runtime files transferred"
echo

# ── Step 4: Install service ──────────────────────────────────────────────────
echo -e "${GREEN}[4/4] Installing systemd service...${NC}"

ssh $SSH_OPTS "$ROBOT" "
    cat ${ROBOT_PATH}/arena.service | sudo tee /etc/systemd/system/arena.service > /dev/null && \
    sudo systemctl daemon-reload
"
echo -e "  ${GREEN}✓${NC} Service installed (not auto-started)"
echo

echo -e "${GREEN}Setup complete!${NC}"
echo -e "  SSH:      ssh ${ROBOT_USER}@${ROBOT_IP}"
echo -e "  Control:  cd ${ROBOT_PATH} && ./run.sh <command>"
echo -e "  Deploy:   pixi run deploy"
