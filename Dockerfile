FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
supervisor \
# libmsgpack is the only library that doesn't reflect what's already inside the robot.
# It is necessary for simbridge now, but we may want to drop it at some point.
libmsgpack-dev \
# Booster motion dependencies
libasound2 \
libpulse0 \
libxcursor1 \
libxinerama1 \
libxi6 \
libxrandr2 \
libxss1 \
libxxf86vm1 \
libdrm2 \
libgbm1 \
libwayland-egl1  \
libwayland-client0  \
libwayland-cursor0  \
libxkbcommon0  \
libdecor-0-0  \
&& rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

COPY external/simbridge/msg/ /bridge_ws/msg/
COPY external/simbridge/src/ /bridge_ws/src/
COPY external/simbridge/tools/ /bridge_ws/tools/
COPY external/simbridge/CMakeLists.txt /bridge_ws/

RUN mv /bridge_ws/tools/booster_motion /opt/booster
# Required by booster-motion command_manager_v2; missing → idle mode may produce zero torques → robot falls.
# COPY sim/system_settings_config.yaml /opt/booster/configs/system_settings_config.yaml
RUN source /opt/ros/humble/setup.bash && \
    cmake -S /bridge_ws -B /tmp/build -DCMAKE_INSTALL_PREFIX=/opt/ros/humble && \
    cmake --build /tmp/build && rm -rf /tmp/build

ENV LD_LIBRARY_PATH=/opt/booster/lib:/opt/booster/lib-usr-local:/opt/booster/lib-x86_64-linux-gnu
ENV BOOSTER_ROOT=/opt/booster

ENTRYPOINT ["/workspace/scripts/docker_entrypoint.sh"]
