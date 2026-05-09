#!/usr/bin/env bash
set -e
source /ros_entrypoint.sh

export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/fastdds_profile.xml
export MODELS_DIR="/workspace/models"

exec /usr/bin/supervisord -n -c /workspace/scripts/supervisord.conf
