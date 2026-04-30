#!/usr/bin/env bash
set -e
export MODELS_DIR="/workspace/external/colosseum/models"
exec /workspace/build/x86/main \
    --backend circus \
    --task t1-velocity-flat \
    --host "${SERVER_IP:-172.21.0.1}" \
    --port "${CIRCUS_PORT:-5555}" \
    --robot "${ROBOT_NAME:-T1}"
