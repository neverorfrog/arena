#!/bin/bash
# One-time robot setup: transfer sysroot, models, sounds, install services.
# The robot is reached at -i IP (default 192.168.10.102).
# When -r NAME is given, the robot's network identity is configured via profile.sh
# (adds 192.168.<team>.<num> as secondary IP on eth0; factory IP is preserved).
set -e

GREEN='\033[0;32m' BLUE='\033[0;34m' YELLOW='\033[1;33m' RED='\033[0;31m' NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/arena_profile.sh"

ROBOT_USER="${ROBOT_USER:-booster}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_PATH="${ROBOT_PATH:-~/spqr/arena}"
ROBOT_PASSWORD="${ROBOT_PASSWORD:-123456}"
ROBOT_NAME=""
TARGET_IP=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--robot)  ROBOT_NAME="$2";  shift 2 ;;
        -i|--ip)     ROBOT_IP="$2";    shift 2 ;;
        -u|--user)   ROBOT_USER="$2";  shift 2 ;;
        -h|--help)
            echo "Usage: $0 [-i IP] [-r NAME] [-u USER]"
            echo "  -i, --ip IP     How to reach the robot now (default: 192.168.10.102)"
            echo "  -r, --robot NAME Assign static wired IP from profile (192.168.19.N)"
            echo "  -u, --user USER SSH username (default: booster)"
            exit 0 ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; exit 1 ;;
    esac
done

if [ -n "$ROBOT_NAME" ]; then
    if ! grep -q "^${ROBOT_NAME} " "$PROFILES_FILE" 2>/dev/null; then
        echo "${ROBOT_NAME} 1" >> "$PROFILES_FILE"
        echo -e "${YELLOW}Profile '$ROBOT_NAME' created (default number=1)${NC}"
    fi
    connect_ip="$ROBOT_IP"
    arena_resolve "$ROBOT_NAME" || exit 1
    TARGET_IP="$ROBOT_IP"
    ROBOT_IP="$connect_ip"
fi

ROBOT="${ROBOT_USER}@${ROBOT_IP}"
arena_ensure_subnet "$ROBOT_IP"
ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
SYSROOT="$ROOT/.pixi/aarch64-sysroot"
SSH_KEY="$HOME/.ssh/id_ed25519"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=5 -i $SSH_KEY"
RSUDO="echo '$ROBOT_PASSWORD' | sudo -S -p ''"

echo -e "${BLUE}=== Arena Robot Setup → ${ROBOT}:${ROBOT_PATH} ===${NC}"
if [ -n "$TARGET_IP" ]; then
    echo -e "${BLUE}  After setup, SSH at: ${ROBOT_USER}@${TARGET_IP}${NC}"
fi
echo

# ── Check sysroot ─────────────────────────────────────────────────────────────
[ -d "$SYSROOT/lib" ] || { echo -e "${RED}aarch64 sysroot not found. Run 'pixi run setup' first.${NC}"; exit 1; }
echo -e "${GREEN}[1/5]${NC} Sysroot ready"

# ── SSH key ───────────────────────────────────────────────────────────────────
if [ ! -f "$SSH_KEY" ]; then
    ssh-keygen -t ed25519 -f "$SSH_KEY" -N "" -C "arena@$(hostname)" >/dev/null 2>&1
fi
if ! ssh $SSH_OPTS -o BatchMode=yes "$ROBOT" exit 2>/dev/null; then
    echo -e "${YELLOW}Copying SSH key to robot...${NC}"
    ssh-copy-id \
        -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o LogLevel=ERROR -o ConnectTimeout=5 \
        -i "$SSH_KEY" "$ROBOT" 2>/dev/null || true
fi

# ── Sudoers ───────────────────────────────────────────────────────────────────
SUDOERS="${ROBOT_USER} ALL=(ALL) NOPASSWD: /usr/bin/systemctl *arena*, /usr/bin/systemctl daemon-reload, /usr/bin/tee /etc/systemd/system/arena*.service, /usr/bin/tee /etc/robot_identity.yaml, /usr/bin/nmcli *, /usr/bin/cp *, /usr/bin/rm *"
ssh $SSH_OPTS "$ROBOT" "
    echo '$SUDOERS' > /tmp/arena_sudoers && \
    $RSUDO cp /tmp/arena_sudoers /etc/sudoers.d/arena && \
    $RSUDO chmod 440 /etc/sudoers.d/arena && \
    rm /tmp/arena_sudoers
" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Passwordless sudo configured" || echo -e "  ${RED}⚠ sudoers setup failed${NC}"

# ── Transfer sysroot ──────────────────────────────────────────────────────────
echo -e "${GREEN}[2/5]${NC} Transferring sysroot to robot..."
ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}/.pixi/aarch64-sysroot"
rsync -az -e "ssh $SSH_OPTS" --delete \
    --exclude="include" --exclude="share" --exclude="targets" \
    "$SYSROOT/" "$ROBOT:${ROBOT_PATH}/.pixi/aarch64-sysroot/"
echo -e "  ${GREEN}✓${NC} Sysroot transferred"

# ── Transfer runtime files ────────────────────────────────────────────────────
echo -e "${GREEN}[3/5]${NC} Transferring runtime files..."
ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}/models ${ROBOT_PATH}/sounds"

echo -e "  ${BLUE}models...${NC}"
rsync -aqz -e "ssh $SSH_OPTS" \
    "$ROOT/models/" "$ROBOT:${ROBOT_PATH}/models/"

echo -e "  ${BLUE}sounds...${NC}"
rsync -aqz -e "ssh $SSH_OPTS" \
    "$ROOT/sounds/" "$ROBOT:${ROBOT_PATH}/sounds/"

for f in run.sh arena.service arena_start.service arena_stop.service; do
    scp -q $SSH_OPTS "$SCRIPT_DIR/$f" "$ROBOT:${ROBOT_PATH}/$f"
done
ssh $SSH_OPTS "$ROBOT" "chmod +x ${ROBOT_PATH}/run.sh"
echo -e "  ${GREEN}✓${NC} Runtime files transferred"

# ── Install services ──────────────────────────────────────────────────────────
echo -e "${GREEN}[4/5]${NC} Installing services..."
ssh $SSH_OPTS "$ROBOT" "
    for f in arena arena_start arena_stop; do
        $RSUDO cp ${ROBOT_PATH}/\$f.service /etc/systemd/system/\$f.service
    done
    $RSUDO systemctl daemon-reload
    $RSUDO systemctl enable --now arena_start arena_stop
"
echo -e "  ${GREEN}✓${NC} Services installed (arena requires joystick activation: RB+RT+A)"

# ── Network identity (only when -r is used) ───────────────────────────────────
if [ -n "$ROBOT_NAME" ]; then
    echo -e "${GREEN}[5/5]${NC} Configuring robot network identity..."
    "$SCRIPT_DIR/profile.sh" set "$ROBOT_NAME" \
        "$(awk -v n="$ROBOT_NAME" '$1==n{print $2}' "$PROFILES_FILE")" \
        --team "$(awk -v n="$ROBOT_NAME" -v dt="$DEFAULT_TEAM_NUMBER" '$1==n{print ($3!=""?$3:dt)}' "$PROFILES_FILE")" \
        -i "$ROBOT_IP"
    arena_ensure_subnet "$TARGET_IP"
    echo -e "  ${GREEN}✓${NC} Identity set — robot also reachable at ${ROBOT_USER}@${TARGET_IP}"
else
    echo -e "${YELLOW}[5/5]${NC} Skipping network identity (use -r to assign one)"
fi

echo
echo -e "${GREEN}Setup complete!${NC}"
echo -e "  ${BLUE}SSH:${NC}      ssh ${ROBOT_USER}@${TARGET_IP:-$ROBOT_IP}"
echo -e "  ${BLUE}Control:${NC}  cd ${ROBOT_PATH} && ./run.sh <command>"
echo -e "  ${BLUE}Deploy:${NC}   pixi run deploy"
