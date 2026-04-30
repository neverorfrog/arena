#!/usr/bin/env bash
set -e
exec /workspace/build/src/main \
    --backend circus \
    --task t1-velocity-flat \
    --host "${SERVER_IP:-172.17.0.1}" \
    --port "${CIRCUS_PORT:-5555}" \
    --robot "${ROBOT_NAME:-T1}"
