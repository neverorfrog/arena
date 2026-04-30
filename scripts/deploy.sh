#!/bin/bash
# Deploy arena to the robot.
# Prerequisites: run scripts/pack_channel.sh first (produces /tmp/bt-aarch64.tar and local-channel/).
#
# Options:
#   -i, --ip IP         Robot IP address (default: 192.168.10.102)
#   -u, --user USER     SSH username    (default: booster)
#   --path PATH         Install path on robot (default: ~/arena)
#   --skip-build        Skip rattler-build step (reuse last output/linux-aarch64/arena-*.conda)
#   --skip-install      Skip environment reassembly on robot

set -e

GREEN='\033[0;32m' BLUE='\033[0;34m' YELLOW='\033[1;33m' RED='\033[0;31m' NC='\033[0m'

ROBOT_USER="${ROBOT_USER:-booster}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_PATH="${ROBOT_PATH:-~/arena}"
SKIP_BUILD=false
SKIP_INSTALL=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--ip)         ROBOT_IP="$2";   shift 2 ;;
        -u|--user)       ROBOT_USER="$2"; shift 2 ;;
        --path)          ROBOT_PATH="$2"; shift 2 ;;
        --skip-build)    SKIP_BUILD=true;   shift ;;
        --skip-install)  SKIP_INSTALL=true; shift ;;
        -h|--help)       head -n 10 "$0" | tail -n +2 | sed 's/^# \?//'; exit 0 ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; exit 1 ;;
    esac
done

ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
SSH_KEY="$HOME/.ssh/id_ed25519"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -i $SSH_KEY"
ROBOT="${ROBOT_USER}@${ROBOT_IP}"

AARCH64_ENV_TAR="$ROOT/local-channel/aarch64-environment.tar"  # produced by pack_channel.sh
BUILD_OUTPUT="$ROOT/output/linux-aarch64"     # rattler-build default output
PIXI_UNPACK_BIN="$ROOT/scripts/bin/pixi-unpack-aarch64"

echo -e "${BLUE}=== Arena Deploy → ${ROBOT}:${ROBOT_PATH} ===${NC}\n"

# ─── Step 1: Build ─────────────────────────────────────────────────────────────
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${GREEN}[1/4] Building arena (linux-aarch64)...${NC}"
    [ -f "$AARCH64_ENV_TAR" ] || { echo -e "${RED}local-channel/aarch64-environment.tar not found — run scripts/pack_channel.sh first${NC}"; exit 1; }
    pixi run --frozen -e arena build
    CONDA_PKG=$(ls "$BUILD_OUTPUT"/arena-*.conda 2>/dev/null | tail -1)
    [ -n "$CONDA_PKG" ] || { echo -e "${RED}No arena-*.conda found in $BUILD_OUTPUT${NC}"; exit 1; }
    echo -e "  ${GREEN}✓${NC} $(basename "$CONDA_PKG") ($(du -h "$CONDA_PKG" | cut -f1))"
else
    CONDA_PKG=$(ls "$BUILD_OUTPUT"/arena-*.conda 2>/dev/null | tail -1)
    [ -n "$CONDA_PKG" ] || { echo -e "${RED}No cached build in $BUILD_OUTPUT — run without --skip-build${NC}"; exit 1; }
    echo -e "${YELLOW}[1/4] Skipping build — using $(basename "$CONDA_PKG")${NC}"
fi
echo

# ─── SSH key setup ─────────────────────────────────────────────────────────────
setup_ssh() {
    if [ ! -f "$SSH_KEY" ]; then
        ssh-keygen -t ed25519 -f "$SSH_KEY" -N "" -C "arena@$(hostname)" >/dev/null 2>&1
        echo -e "  ${GREEN}✓${NC} SSH key generated"
    fi
    if ! ssh $SSH_OPTS -o PasswordAuthentication=no -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT" exit 2>/dev/null; then
        echo -e "  Copying SSH key to robot (password required once)..."
        ssh-copy-id -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -i "$SSH_KEY" "$ROBOT"
        echo -e "  ${GREEN}✓${NC} SSH key installed"
    fi
}

# ─── pixi-unpack binary (downloaded once, cached in scripts/bin/) ───────────────
ensure_pixi_unpack() {
    if [ ! -f "$PIXI_UNPACK_BIN" ]; then
        echo -e "  Downloading pixi-unpack for aarch64..."
        mkdir -p "$(dirname "$PIXI_UNPACK_BIN")"
        curl -fSL "https://github.com/Quantco/pixi-pack/releases/latest/download/pixi-unpack-aarch64-unknown-linux-musl" \
            -o "$PIXI_UNPACK_BIN"
        chmod +x "$PIXI_UNPACK_BIN"
        echo -e "  ${GREEN}✓${NC} pixi-unpack downloaded"
    fi
}

# ─── Step 2: Transfer ──────────────────────────────────────────────────────────
echo -e "${GREEN}[2/4] Transferring to robot...${NC}"
setup_ssh
ensure_pixi_unpack
ssh $SSH_OPTS "$ROBOT" "mkdir -p ${ROBOT_PATH}"

# Skip retransferring the base env if it hasn't changed
LOCAL_ENV_SUM=$(md5sum "$AARCH64_ENV_TAR" | cut -d' ' -f1)
REMOTE_ENV_SUM=$(ssh $SSH_OPTS "$ROBOT" "cat ${ROBOT_PATH}/.env_checksum 2>/dev/null" || echo '')

if [ "$LOCAL_ENV_SUM" != "$REMOTE_ENV_SUM" ]; then
    echo -e "  Transferring base environment ($(du -h "$AARCH64_ENV_TAR" | cut -f1))..."
    scp $SSH_OPTS "$AARCH64_ENV_TAR"   "$ROBOT:${ROBOT_PATH}/environment.tar"
    scp $SSH_OPTS "$PIXI_UNPACK_BIN"   "$ROBOT:${ROBOT_PATH}/pixi-unpack"
    ssh $SSH_OPTS "$ROBOT" "chmod +x ${ROBOT_PATH}/pixi-unpack"
else
    echo -e "  ${GREEN}✓${NC} Base environment unchanged — skipping"
fi

echo -e "  Transferring arena package and run script..."
scp $SSH_OPTS "$CONDA_PKG"                         "$ROBOT:${ROBOT_PATH}/arena.conda"
scp $SSH_OPTS "$(dirname "$0")/run.sh"             "$ROBOT:${ROBOT_PATH}/run.sh"
ssh $SSH_OPTS "$ROBOT" "chmod +x ${ROBOT_PATH}/run.sh"
echo -e "  ${GREEN}✓${NC} Transfer complete ($(du -h "$CONDA_PKG" | cut -f1))"
echo

# ─── Step 3: Reassemble environment on robot ───────────────────────────────────
echo -e "${GREEN}[3/4] Reassembling environment on robot...${NC}"
if [ "$SKIP_INSTALL" = false ]; then

    if [ "$LOCAL_ENV_SUM" != "$REMOTE_ENV_SUM" ]; then
        echo -e "  Unpacking base environment..."
        ssh $SSH_OPTS "$ROBOT" "cd ${ROBOT_PATH} && rm -rf env activate.sh && ./pixi-unpack environment.tar && echo '${LOCAL_ENV_SUM}' > .env_checksum"
        echo -e "  ${GREEN}✓${NC} Base environment unpacked"
    else
        echo -e "  ${GREEN}✓${NC} Base environment unchanged"
    fi

    echo -e "  Installing arena package..."
    ssh $SSH_OPTS "$ROBOT" "cd ${ROBOT_PATH} && \
        pkg_entry=\$(unzip -Z1 arena.conda | grep '^pkg-') && \
        info_entry=\$(unzip -Z1 arena.conda | grep '^info-') && \
        unzip -p arena.conda \"\$pkg_entry\"  | zstd -d | tar -xC env && \
        unzip -p arena.conda \"\$info_entry\" | zstd -d | tar -xC env"
        echo -e "  ${GREEN}✓${NC} arena installed"

else
    echo -e "${YELLOW}[3/4] Skipping install (--skip-install)${NC}"
fi
echo

# ─── Step 4: Done ──────────────────────────────────────────────────────────────
echo -e "${GREEN}[4/4] Done.${NC}"
echo -e "  Connect: ssh ${ROBOT_USER}@${ROBOT_IP}"
echo -e "  Run    : cd ${ROBOT_PATH} && ./run.sh run"
