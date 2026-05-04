# Buildchain & Sim Integration Notes

Notes from analysis of `spqrbooster2026` (`buildchain-rework` branch, starting from commit
`f12a01d`) and `circus`, with conclusions on how to replicate/adapt for `colosseum/src/arena/`.

---

## 1. SPQRBooster2026 Buildchain

### 1.1 Overview

There are three independent execution modes, all sharing one CMake source tree:

| Mode | How | Output |
|------|-----|--------|
| x86 native | `pixi run compile` | `build/x86/src/app/maximus_main` |
| aarch64 cross-compile | `pixi run compile-aarch64` | `build/aarch64/src/app/maximus_main` |
| Docker sim2sim | `pixi run containers` | runs x86 binary against `booster-motion` in container |

There is **no conda packaging** in the build pipeline. `build.sh` / `deploy.sh` are dead weight
from an earlier approach and can be ignored. Everything relevant is in `pixi.toml` and
`cmake/aarch64-toolchain.cmake`.

---

### 1.2 Dependency Management: Pixi

`pixi.toml` defines two environments:

```
default  →  x86 development (cmake, ninja, yaml-cpp, eigen, booster SDK, rerun, CUDA, …)
robot    →  adds gcc_linux-aarch64 / gxx_linux-aarch64 cross-compiler from conda-forge
```

The `robot` environment is only activated to run the cross-compilation tasks. All other
development uses `default`.

---

### 1.3 One-Time Setup: `scripts/setup.sh`

Run once per machine. Does in order:

1. `git submodule update --init --recursive`
2. Install Docker if missing
3. Detect GPU (RTX 30/40/50) and download the matching TensorRT tarball from GDrive
4. Download the aarch64 TensorRT tarball as well
5. `docker build -t spqr_maximus_bridge .` — build the sim2sim image
6. `pixi install` — resolves the `default` env
7. `pixi_install_aarch64 robot` — builds the aarch64 sysroot (see §1.5)
8. Symlink TensorRT `.so` files into both envs:
   - x86: `external/TensorRT/x86/…/lib/*.so* → .pixi/envs/default/lib/`
   - aarch64: `external/TensorRT/aarch64/…/lib/*.so* → .pixi/aarch64-sysroot/lib/`

TensorRT is the only dependency not available on conda-forge; it is handled manually.

---

### 1.4 The aarch64 Sysroot: `scripts/pixi_install_aarch64.sh`

Pixi cannot natively download aarch64 packages when running on an x86 host. The workaround:

```bash
# Download all aarch64 packages from the default env as a tarball
pixi exec pixi-pack --environment default --platform linux-aarch64 pixi.toml
# → environment.tar

# Unpack into scratch
tar -xf environment.tar -C /tmp/scratch

# For each .conda in scratch:
#   skip: gcc, gxx, binutils, cmake, cuda-nvcc  (host tools, useless for cross)
#   extract: include/, lib/, share/, targets/  → .pixi/aarch64-sysroot/
#   extract: aarch64-conda-linux-gnu/sysroot/  → .pixi/aarch64-sysroot/
```

Two fixup symlinks after extraction:
- `targets/sbsa-linux → targets/aarch64-linux` (Jetson CUDA convention)
- `lib/ld-linux-aarch64.so.1 → lib64/ld-linux-aarch64.so.1` (standard Linux path)

Result: `.pixi/aarch64-sysroot/` is a self-contained directory with all aarch64 headers and
shared library stubs needed to cross-compile and link.

---

### 1.5 Toolchain File: `cmake/aarch64-toolchain.cmake`

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)  # don't execute compiled tests on x86

# Wipe pixi's x86 CFLAGS/LDFLAGS — invalid for aarch64
foreach(_env_var CFLAGS CXXFLAGS CPPFLAGS LDFLAGS LDFLAGS_LD)
    unset(ENV{${_env_var}})
endforeach()

set(CMAKE_C_COMPILER   aarch64-conda-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-conda-linux-gnu-g++)
set(CMAKE_C_FLAGS   "-Wno-psabi")
set(CMAKE_CXX_FLAGS "-Wno-psabi")

set(CMAKE_SYSROOT        $ENV{PIXI_PROJECT_ROOT}/.pixi/aarch64-sysroot)
set(CMAKE_FIND_ROOT_PATH $ENV{PIXI_PROJECT_ROOT}/.pixi/aarch64-sysroot)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)  # host tools stay on host PATH
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

All `find_package` / `find_library` calls resolve into the sysroot, not the host system.

---

### 1.6 Cross-Compilation Tasks in `pixi.toml`

```toml
[feature.cross-compile.tasks]
_configure-aarch64 = { cmd = "cmake -S $PIXI_PROJECT_ROOT -B .../build/aarch64
                               -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
                               --toolchain cmake/aarch64-toolchain.cmake" }
_compile-aarch64   = { cmd = "cmake --build .../build/aarch64 --parallel 8" }

# Exposed as top-level (run inside the robot env):
configure-aarch64 = { cmd = "pixi run -e robot _configure-aarch64" }
compile-aarch64   = { cmd = "pixi run -e robot _compile-aarch64"   }
```

`pixi run -e robot` activates the `robot` env so `aarch64-conda-linux-gnu-g++` is on `$PATH`.
Output: `build/aarch64/src/app/maximus_main` — a native aarch64 ELF binary.

---

### 1.7 Root CMakeLists.txt: The glibc Trick (x86 only)

```cmake
if(DEFINED ENV{CONDA_PREFIX} AND NOT CMAKE_CROSSCOMPILING)
    set(_sysroot "$ENV{CONDA_PREFIX}/x86_64-conda-linux-gnu/sysroot")
    set(CMAKE_BUILD_RPATH "${_sysroot}/lib64;$ENV{CONDA_PREFIX}/lib")
    string(APPEND CMAKE_EXE_LINKER_FLAGS
        " -Wl,-dynamic-linker,${_sysroot}/lib64/ld-linux-x86-64.so.2")
endif()
```

Forces the binary to use the conda sysroot's dynamic linker at runtime, avoiding glibc
version mismatches between pixi dependencies and the host Ubuntu installation. Not needed
for cross-compilation (the sysroot already provides the correct aarch64 linker).

---

### 1.8 Docker (sim2sim)

**`Dockerfile`** (`FROM ros:humble`):
- Installs `libmsgpack-dev` and Booster Motion runtime deps (audio/display libs)
- Copies `external/simbridge/` into `/bridge_ws/` and builds it against ROS2 Humble
- Copies `booster-motion` binary to `/opt/booster/`

**`scripts/run_container.sh`**:
```bash
docker run -it --rm \
  --gpus all                  # GPU passthrough (if available)
  --cap-add=sys_nice          # real-time scheduling
  --ipc=host                  # shared memory for camera frames
  -v "$(pwd)":/workspace      # binary read from mounted workspace, not baked in image
  spqr_maximus_bridge \
  /workspace/scripts/docker_entrypoint.sh
```

**`scripts/docker_entrypoint.sh`**:
```bash
source /ros_entrypoint.sh                                   # ROS2 env
cd /opt/booster && ./booster-motion -mode sim … &           # start Circus sim layer
"${PIXI_SYSROOT}/lib64/ld-linux-x86-64.so.2" \
    --library-path "…" \
    /workspace/build/x86/src/app/maximus_main &             # start framework (glibc trick)
wait -n
```

Key: the binary is **not baked into the image**. It is read from the mounted workspace, so
rebuilding with `pixi run compile` takes effect immediately without rebuilding the image.

---

### 1.9 Full Developer Flow Summary

```
# One-time setup
pixi run setup

# Sim2sim development loop
pixi run compile       → build/x86/src/app/maximus_main
pixi run containers    → docker run (mounts workspace, starts booster-motion + maximus_main)

# Robot deployment
pixi run compile-aarch64   → build/aarch64/src/app/maximus_main
# rsync to robot + systemd
```

---

## 2. Circus Architecture

### 2.1 What Circus Is

Circus is a MuJoCo-based soccer simulator. It is a standalone GUI application (Qt6) that:
- Loads a scene file defining robots, teams, and field
- Runs the MuJoCo physics loop
- Acts as a TCP server: each robot connects and exchanges state/commands via msgpack
- Manages Docker containers (one per robot) via the Docker API

It has Python bindings (`circuspy` via nanobind) but they are not currently used by the
consuming projects.

---

### 2.2 How N Containers Are Spawned

Circus talks directly to the **Docker daemon via its Unix socket** (`/var/run/docker.sock`)
using libcurl (`DockerREST.h` → `CURLClient`). No `docker` CLI involved.

Flow when a scene is loaded:

1. **Parse scene** (`1v1.yaml`) → builds N Robot objects
2. **`RobotManager::startContainers()`** iterates over robots:
   ```cpp
   for (auto r : robots_) {
       r->container = make_unique<Container>("CIRCUS_" + r->name + "_container");
       r->container->create(r, image, binds);  // POST /containers/create
       r->container->start();                   // POST /containers/{id}/start
   }
   ```
3. **`Container::create()`** sends a JSON payload to `/containers/create`:
   - Image: from `framework_config.yaml`
   - Volumes: from `framework_config.yaml` (resolved via `path_constants.yaml`)
   - Env injected automatically: `ROBOT_NAME`, `SERVER_IP=172.17.0.1`, `CIRCUS_PORT`,
     `TEAM_NUMBER`, `PLAYER_NUMBER`, `TEAM_COLOR`
   - Network: `CIRCUS_network` (Docker bridge), deterministic IP per robot
   - `--ipc=host`, GPU passthrough, `--privileged`
4. **`CircusNetwork`** (singleton) creates the Docker bridge network on startup,
   tears it down on destruction

`1v1.yaml` (2 robots) → 2 containers. `5v5.yaml` (10 robots) → 10 containers.

---

### 2.3 Communication Chain (spqrbooster2026)

Inside each spawned container, `supervisord` manages three processes:

```
maximus_main  ←→  DDS/Fast-DDS  ←→  booster-motion  ←→  ROS2 topics  ←→  simbridge  ←→  TCP/msgpack  ←→  Circus (host)
```

- `booster-motion`: DDS broker — maximus thinks it's on a real robot
- `simbridge`: ROS2 ↔ TCP bridge, subscribes to joint command topics, forwards to Circus,
  receives sim state, publishes back to ROS2
- `maximus_main`: the framework, never aware it is in simulation

---

### 2.4 Communication Chain (colosseum/arena)

Arena's `CircusPortal` speaks TCP/msgpack **directly** to Circus. No `booster-motion`,
no `simbridge`, no ROS2 needed:

```
arena (--backend circus)  ←→  CircusPortal  ←→  TCP/msgpack  ←→  Circus (host)
```

The container image for arena is much simpler: just the arena binary + ONNX Runtime.

---

## 3. The Dependency Problem and Its Solution

### 3.1 The Current Problem

Circus is currently the **orchestrator**, not a dependency. It knows about its consumers via:
- `path_constants.yaml`: hardcoded absolute paths to `spqrbooster2026` and `simbridge`
- `framework_config.yaml`: mounts volumes from those paths into each container
- Scene files (`1v1.yaml`, `5v5.yaml`): live in circus, not in the consuming project

This means every time the consuming project changes (new image, new volume, new binary path),
circus must be updated. Circus cannot be used by colosseum without duplicating this config.
The dependency is inverted from what it should be.

---

### 3.2 The Correct Design

Circus should be a **pure physics server**: given a scene and a container spec, run the
physics and spawn the agents. It should not know who is consuming it.

The two concerns currently tangled together must be separated:

| Concern | Owner | What it describes |
|---------|-------|-------------------|
| Physics scene | **consuming project** | N robots, types, positions, field config |
| Container spec | **consuming project** | image, volumes, env vars per robot |
| Robot type → MJCF | **circus** | `Booster-T1 → robots/booster_t1.xml` |
| Physics/sim params | **circus** | timestep, gravity, rendering |

---

### 3.3 Proposed Mechanism

Circus accepts two CLI arguments:

```bash
circus --scene /path/to/scene.yaml --framework /path/to/framework_config.yaml
```

Both files are owned by the consuming project. `path_constants.yaml` is deleted from circus.

**`scene.yaml`** (consuming project, describes physics):
```yaml
simulation_config: default
teams:
  red:
    players:
      - type: Booster-T1
        position: [0, 0, 0.68]
        orientation: [0, 0, 0]
```

**`framework_config.yaml`** (consuming project, describes containers):
```yaml
image: colosseum/arena
volumes:
  - /home/user/colosseum/build/aarch64/arena:/arena
  - /home/user/colosseum/models:/models
```

Circus reads both, counts N robots from the scene, and for each robot spawns the container
described in `framework_config.yaml`, injecting `ROBOT_NAME`, `SERVER_IP`, `CIRCUS_PORT`
automatically. It never hardcodes anything about maximus or colosseum.

---

### 3.4 Pixi Tasks in the Consuming Project

**spqrbooster2026:**
```toml
sim-1v1 = { cmd = "circus --scene $PIXI_PROJECT_ROOT/sim/1v1.yaml --framework $PIXI_PROJECT_ROOT/sim/framework_config.yaml" }
sim-5v5 = { cmd = "circus --scene $PIXI_PROJECT_ROOT/sim/5v5.yaml --framework $PIXI_PROJECT_ROOT/sim/framework_config.yaml" }
```

**colosseum:**
```toml
sim = { cmd = "circus --scene $PIXI_PROJECT_ROOT/sim/scene.yaml --framework $PIXI_PROJECT_ROOT/sim/framework_config.yaml" }
```

Development iterates entirely in the consuming project repo. Circus is infrastructure,
treated like a binary you install and invoke — the same relationship as MuJoCo or Gazebo.

---

### 3.5 Why This Pattern Is Standard

This is exactly how Gazebo/ROS2 works:
- `gazebo` accepts a `.world` file (scene) and has no idea what nodes will connect to it
- Your project's launch file passes the world file and starts your nodes
- Gazebo has zero knowledge of your stack

Circus would reach the same design with two changes:
1. Accept `--scene` and `--framework` as CLI args
2. Delete `path_constants.yaml`

---

## 4. Colosseum / Arena Specifics

### 4.1 What Arena Already Has

`src/arena/` is the C++ deployment harness for colosseum. It already implements:
- `CircusPortal`: direct TCP/msgpack connection to Circus (no simbridge needed)
- `MujocoPortal`: local MuJoCo viewer for development
- `RobotPortal`: real Booster T1 via DDS/SDK
- ONNX Runtime policy inference
- Task registry (`REGISTER_TASK` macro)

### 4.2 Buildchain Adaptations Needed

To replicate the spqrbooster2026 buildchain for arena:

1. **`cmake/aarch64-toolchain.cmake`**: essentially identical, copy and adjust sysroot path
2. **`pixi.toml` `robot` feature**: add `gcc_linux-aarch64` + cross-compile tasks pointing to
   `build/aarch64/`
3. **`scripts/pixi_install_aarch64.sh`**: copy verbatim (it is generic)
4. **Sysroot contents**: arena's deps are simpler — ONNX Runtime aarch64 + Booster SDK aarch64
   (no CUDA/TensorRT needed unless future policy inference requires it)
5. **Docker image**: simpler than spqrbooster2026's — no ROS2, no simbridge, no booster-motion;
   just arena binary + ONNX Runtime `.so` files + Booster SDK libs if needed

### 4.3 Sim2Sim Mode

For colosseum's sim2sim:
- Circus spawns one container running `arena --backend circus --task T1VelocityFlat`
- `CircusPortal` connects to `SERVER_IP:CIRCUS_PORT` (injected by Circus as env vars)
- No intermediary processes needed

The arena Docker image would be built once and referenced in colosseum's `framework_config.yaml`.
The binary inside it can be volume-mounted from `build/x86/arena` (for fast iteration without
rebuilding the image), mirroring spqrbooster2026's approach.
