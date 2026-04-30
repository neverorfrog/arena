#!/usr/bin/env bash
set -e
source /ros_entrypoint.sh

# Paths
PIXI_SYSROOT="/workspace/.pixi/envs/default/x86_64-conda-linux-gnu/sysroot"
PIXI_LIB="/workspace/.pixi/envs/default/lib"
BOOSTER_LIB="/opt/booster/lib"
BOOSTER_USR_LOCAL="/opt/booster/lib-usr-local"
BOOSTER_SYSTEM="/opt/booster/lib-x86_64-linux-gnu"

# Environment
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/fastdds_profile.xml
export MODELS_DIR="/workspace/external/colosseum/models"

# booster-motion & simbridge use LD_LIBRARY_PATH to find bundled libs
export LD_LIBRARY_PATH="${BOOSTER_LIB}:${BOOSTER_USR_LOCAL}:${BOOSTER_SYSTEM}:${PIXI_LIB}:${LD_LIBRARY_PATH:-}"

# arena uses pixi sysroot's ld-linux for glibc isolation.
# CRITICAL: booster-motion's lib-usr-local must come BEFORE pixi_lib so arena
# loads the SAME libfastrtps.so as booster-motion. Two different FastDDS
# builds in the same DDS domain will not discover each other.
LIBRARY_PATH="${PIXI_SYSROOT}/lib64:${BOOSTER_USR_LOCAL}:${BOOSTER_LIB}:${PIXI_LIB}:${BOOSTER_SYSTEM}:${LD_LIBRARY_PATH:-}"

# 1. booster-motion
cd /opt/booster && ./booster-motion -mode sim -config configs/config_isaac.lua &
/opt/ros/humble/bin/simbridge &
sleep 2

# 3. arena
"${PIXI_SYSROOT}/lib64/ld-linux-x86-64.so.2" \
    --library-path "${LIBRARY_PATH}" \
    /workspace/build/x86/main --backend booster --task t1-velocity-flat &

wait -n
