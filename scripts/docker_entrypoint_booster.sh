#!/usr/bin/env bash
set -e
source /ros_entrypoint.sh
cd /opt/booster && ./booster-motion -mode sim -config configs/config_isaac.lua &
PIXI_SYSROOT="/workspace/.pixi/envs/default/x86_64-conda-linux-gnu/sysroot"
LIBRARY_PATH="${PIXI_SYSROOT}/lib64:/workspace/.pixi/envs/default/lib:/workspace/external/TensorRT/x86/targets/x86_64-linux-gnu/lib"
"${PIXI_SYSROOT}/lib64/ld-linux-x86-64.so.2" \
    --library-path "${PIXI_SYSROOT}/lib64:/workspace/.pixi/envs/default/lib" \
    /workspace/build/src/main --backend booster --task t1-velocity-flat &
wait -n
