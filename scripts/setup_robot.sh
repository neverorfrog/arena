#!/bin/bash
# One-time robot setup: transfer sysroot, models, sounds, and install services.
set -e

GREEN='\033[0;32m' BLUE='\033[0;34m' YELLOW='\033[1;33m' RED='\033[0;31m' NC='\033[0m'

ROBOT_USER="${ROBOT_USER:-booster}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_PATH="${ROBOT_PATH:-~/spqr/arena}"
ROBOT_PASSWORD="${ROBOT_PASSWORD:-123456}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
SYSROOT="$ROOT/.pixi/aarch64-sysroot"
SSH_KEY="$HOME/.ssh/id_ed25519"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -i $SSH_KEY"
ROBOT="${ROBOT_USER}@${ROBOT_IP}"
RSUDO="echo '$ROBOT_PASSWORD' | sudo -S -p ''"

echo -e "${BLUE}=== Arena Robot Setup → ${ROBOT}:${ROBOT_PATH} ===${NC}"

# ── Check sysroot ─────────────────────────────────────────────────────────────
[ -d "$SYSROOT/lib" ] || { echo -e "${RED}aarch64 sysroot not found. Run 'pixi run setup' first.${NC}"; exit 1; }
echo -e "${GREEN}[1/4]${NC} Sysroot ready"

# ── SSH key ───────────────────────────────────────────────────────────────────
if [ ! -f "$SSH_KEY" ]; then
    ssh-keygen -t ed25519 -f "$SSH_KEY" -N "" -C "arena@$(hostname)" >/dev/null 2>&1
fi
if ! ssh $SSH_OPTS -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT" exit 2>/dev/null; then
    echo -e "${YELLOW}Copying SSH key to robot...${NC}"
    ssh-copy-id -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -i "$SSH_KEY" "$ROBOT" 2>/dev/null
fi

# ── Sudoers ───────────────────────────────────────────────────────────────────
if ! ssh $SSH_OPTS "$ROBOT" "sudo -n true" 2>/dev/null; then
    echo -e "${YELLOW}Configuring passwordless sudo...${NC}"
    SUDOERS="${ROBOT_USER} ALL=(ALL) NOPASSWD: /usr/bin/systemctl *arena*, /usr/bin/systemctl daemon-reload, /usr/bin/tee /etc/systemd/system/arena*.service, /usr/bin/cp *, /usr/bin/rm *"
    if ssh $SSH_OPTS "$ROBOT" "
        echo '$SUDOERS' > /tmp/arena_sudoers && \
        $RSUDO cp /tmp/arena_sudoers /etc/sudoers.d/arena && \
        $RSUDO chmod 440 /etc/sudoers.d/arena && \
        rm /tmp/arena_sudoers
    " 2>/dev/null; then
        echo -e "  ${GREEN}✓${NC} Passwordless sudo configured"
    else
        echo -e "  ${RED}⚠ sudoers setup failed${NC}"
    fi
fi

# ── Transfer sysroot ──────────────────────────────────────────────────────────
echo -e "${GREEN}[2/4]${NC} Transferring sysroot to robot..."
ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}/.pixi/aarch64-sysroot"
rsync -az -e "ssh $SSH_OPTS" --delete \
    --exclude="include" --exclude="share" --exclude="targets" \
    "$SYSROOT/" "$ROBOT:${ROBOT_PATH}/.pixi/aarch64-sysroot/"
echo -e "  ${GREEN}✓${NC} Sysroot transferred"

# ── Transfer runtime files ────────────────────────────────────────────────────
echo -e "${GREEN}[3/4]${NC} Transferring runtime files..."
ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}/models ${ROBOT_PATH}/sounds"

echo -e "  ${BLUE}models...${NC}"
rsync -aqz -e "ssh $SSH_OPTS" \
    "$ROOT/external/colosseum/models/" "$ROBOT:${ROBOT_PATH}/models/"

echo -e "  ${BLUE}sounds...${NC}"
rsync -aqz -e "ssh $SSH_OPTS" \
    "$ROOT/sounds/" "$ROBOT:${ROBOT_PATH}/sounds/"

for f in run.sh arena.service arena_start.service arena_stop.service; do
    scp -q $SSH_OPTS "$SCRIPT_DIR/$f" "$ROBOT:${ROBOT_PATH}/$f"
done
ssh $SSH_OPTS "$ROBOT" "chmod +x ${ROBOT_PATH}/run.sh"

echo -e "  ${GREEN}✓${NC} Runtime files transferred"

# ── Install services ──────────────────────────────────────────────────────────
echo -e "${GREEN}[4/4]${NC} Installing services..."
ssh $SSH_OPTS "$ROBOT" "
    for f in arena arena_start arena_stop; do
        $RSUDO cp ${ROBOT_PATH}/\$f.service /etc/systemd/system/\$f.service
    done
    $RSUDO systemctl daemon-reload
    $RSUDO systemctl enable --now arena_start arena_stop
"
echo -e "  ${GREEN}✓${NC} Services installed (arena requires joystick activation: RB+RT+A)"

echo
echo -e "${GREEN}Setup complete!${NC}"
echo -e "  ${BLUE}SSH:${NC}      ssh ${ROBOT_USER}@${ROBOT_IP}"
echo -e "  ${BLUE}Control:${NC}  cd ${ROBOT_PATH} && ./run.sh <command>"
echo -e "  ${BLUE}Deploy:${NC}   pixi run deploy"
