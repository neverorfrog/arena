# Plan: Buildchain & Sim Integration

## Context

The document `docs/BUILDCHAIN_AND_SIM_INTEGRATION.md` describes two concrete engineering tasks:

1. **Arena buildchain** — add aarch64 cross-compilation + Docker sim2sim support + TensorRT inference backend alongside ONNX Runtime. Everything arena-related lives in `src/arena/`; arena is independent of colosseum.
2. **Circus dependency inversion** — add `--scene <path>` and `--framework <path>` CLI args. All consuming projects migrate to this pattern. Circus becomes pure infrastructure.

Additional deliverables:

3. **Policy contract** — `policies/` at repo root: language-agnostic YAML spec shared between Python training (colosseum) and C++ deployment (arena).
4. **Remove Python deploy dead code** — `deploy.py` was never written; `src/colosseum/utils/deploy/` and the `deploy` pixi environment are obsolete.

---

## Pre-condition: `--task` flag already works

`src/arena/src/main.cpp:34` already parses `--task <name>` via `TaskRegistry`. No changes needed.

---

## Three Arena Execution Scenarios

| Scenario | Backend | How | When |
|----------|---------|-----|------|
| **A** Plain sim2sim | `--backend circus` | Arena ↔ TCP/msgpack ↔ Circus directly | Fast sim iteration |
| **B** DDS sim2sim | `--backend booster` | Arena → DDS → booster-motion → simbridge → Circus | Full DDS path validation |
| **C** Real robot | `--backend booster` | Arena → DDS → actual hardware | Production |

---

## Part 1: Arena Buildchain

### Repository layout after implementation

```
src/arena/
├── cmake/
│   └── aarch64-toolchain.cmake             ← new (copy from spqrbooster2026)
├── scripts/
│   ├── setup.sh                            ← new
│   ├── pixi_install_aarch64.sh             ← new (copy verbatim from spqrbooster2026)
│   ├── deploy.sh                           ← move from repo root scripts/
│   ├── pack_channel.sh                     ← move from repo root scripts/
│   ├── run.sh                              ← move from repo root scripts/
│   ├── docker_entrypoint_circus.sh         ← new
│   └── docker_entrypoint_booster.sh        ← new
├── sim/
│   ├── scene.yaml                          ← new
│   ├── framework_circus.yaml               ← new (Scenario A)
│   └── framework_booster.yaml             ← new (Scenario B)
├── include/
│   ├── IInferenceEngine.h                  ← new (abstract interface)
│   └── TrtEngine.h                         ← new (TensorRT backend)
├── src/
│   └── TrtEngine.cpp                       ← new
├── Dockerfile.circus                       ← new
├── Dockerfile.booster                      ← new
└── ...existing source files...

policies/
└── t1_velocity_flat.yaml                   ← new root-level policy contract
```

### 1a. Inference Engine Abstraction

Currently `Policy` owns `OnnxPolicy onnx` directly (`Policy.h:69`). Extract a common interface:

#### `include/IInferenceEngine.h` — new

```cpp
#pragma once
#include <Eigen/Dense>

class IInferenceEngine {
public:
    virtual ~IInferenceEngine() = default;
    virtual Eigen::VectorXf infer(const Eigen::VectorXf& input) = 0;
    virtual int input_dim()  const = 0;
    virtual int output_dim() const = 0;
};
```

#### `include/OnnxPolicy.h` — add inheritance

```cpp
class OnnxPolicy : public IInferenceEngine {
    // existing implementation unchanged; add "public IInferenceEngine" to class declaration
    ...
};
```

#### `include/TrtEngine.h` — new

```cpp
#pragma once
#include "IInferenceEngine.h"
#include <NvInfer.h>
#include <string>
#include <memory>

class TrtEngine : public IInferenceEngine {
public:
    explicit TrtEngine(const std::string& engine_path);
    Eigen::VectorXf infer(const Eigen::VectorXf& input) override;
    int input_dim()  const override { return input_dim_; }
    int output_dim() const override { return output_dim_; }
private:
    std::unique_ptr<nvinfer1::IRuntime>             runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine>          engine_;
    std::unique_ptr<nvinfer1::IExecutionContext>    context_;
    int input_dim_, output_dim_;
};
```

#### `include/Policy.h` — change `onnx` member to `engine_`

```cpp
// Change line 69: OnnxPolicy onnx; → std::unique_ptr<IInferenceEngine> engine_;
// Change constructor:
explicit Policy(TaskConfig cfg)
    : config_(std::move(cfg)),
      engine_(make_engine(config_)) {
    observation.reserve(engine_->input_dim());
}
// Replace all onnx.* calls with engine_->*
```

Add a factory helper (free function or static method):

```cpp
inline std::unique_ptr<IInferenceEngine> make_engine(const TaskConfig& cfg) {
    if (cfg.inference_backend == "trt") {
        // .engine path derived from model_path (replace .onnx → .engine)
        std::string engine_path = cfg.model_path;
        auto pos = engine_path.rfind(".onnx");
        if (pos != std::string::npos) engine_path.replace(pos, 5, ".engine");
        return std::make_unique<TrtEngine>(engine_path);
    }
    return std::make_unique<OnnxPolicy>(cfg.model_path);
}
```

#### `include/TaskConfig.h` — add inference_backend field

```cpp
struct TaskConfig {
    // ... existing fields ...
    std::string inference_backend = "onnx";  // "onnx" | "trt"
};
```

#### `src/main.cpp` — add `--inference` flag

```cpp
std::string inference_backend = "onnx";
// in arg parsing loop:
if (std::strcmp(argv[i], "--inference") == 0 && i + 1 < argc) inference_backend = argv[++i];
// before creating policy:
cfg.inference_backend = inference_backend;
```

#### `src/arena/CMakeLists.txt` — conditional TRT

```cmake
find_library(TENSORRT_LIB nvinfer)
if(TENSORRT_LIB)
    message(STATUS "TensorRT found — building TrtEngine")
    target_sources(arena PRIVATE src/TrtEngine.cpp)
    target_link_libraries(arena PRIVATE ${TENSORRT_LIB})
    target_compile_definitions(arena PRIVATE WITH_TENSORRT)
else()
    message(STATUS "TensorRT not found — TRT backend disabled")
endif()
```

When `WITH_TENSORRT` is not defined, `make_engine()` throws if `--inference trt` is requested.

### 1b. `setup.sh` — includes TensorRT download

```bash
#!/usr/bin/env bash
set -e
source "$(dirname "$0")/pixi_install_aarch64.sh"
ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

requires() { command -v "$1" >/dev/null 2>&1 || sudo apt-get install -y "${@:2}"; }
requires_docker() {
    requires docker docker.io
    docker info >/dev/null 2>&1 || sudo systemctl start docker
}

detect_gpu() {
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1)
    case "$GPU_NAME" in
        *"RTX 30"*|*"RTX 40"*) ARCHIVE_X86="TensorRT-10.3.0.26.Linux.x86_64-gnu.cuda-12.5.tar.gz" ;;
        *"RTX 50"*)             ARCHIVE_X86="TensorRT-10.13.2.6.Linux.x86_64-gnu.cuda-13.0.tar.gz" ;;
        *) echo "No supported GPU — TensorRT will not be available"; ARCHIVE_X86=""; return ;;
    esac
    ARCHIVE_AARCH64="TensorRT-10.3.0.26.l4t.aarch64-gnu.cuda-12.6.tar.gz"
}

# Same GDrive IDs as spqrbooster2026 (copy FILE_MAP from its setup.sh)
download_tensorrt() { ... }  # same as spqrbooster2026/scripts/setup.sh
extract()           { ... }  # same

build_trt_engines() {
    for model_dir in "$ROOT"/models/*/; do
        local raw_onnx="${model_dir}$(basename "$model_dir").onnx"
        local engine="${model_dir}$(basename "$model_dir").engine"
        [ -f "$raw_onnx" ] || continue
        [ -f "$engine" ] && [ "$engine" -nt "$raw_onnx" ] && continue
        LD_LIBRARY_PATH="$ROOT/external/TensorRT/x86/lib" \
            "$ROOT/external/TensorRT/x86/bin/trtexec" \
            --onnx="$raw_onnx" --saveEngine="$engine" --builderOptimizationLevel=5
    done
}

main() {
    git submodule update --init --recursive
    requires cmake cmake build-essential
    requires_docker
    detect_gpu

    if [ -n "$ARCHIVE_X86" ]; then
        mkdir -p "$ROOT/external/TensorRT"
        [ -d "$ROOT/external/TensorRT/x86" ]    || { download_tensorrt "$ARCHIVE_X86";    extract "$ARCHIVE_X86" "TensorRT/x86"; }
        [ -d "$ROOT/external/TensorRT/aarch64" ] || { download_tensorrt "$ARCHIVE_AARCH64"; extract "$ARCHIVE_AARCH64" "TensorRT/aarch64"; }
        rm -f "$ROOT/external/TensorRT"/*.tar.gz
    fi

    pixi install -e arena
    cd "$ROOT" && pixi_install_aarch64 arena   # creates .pixi/aarch64-sysroot

    # Symlink TRT libs into pixi envs (same as spqrbooster2026)
    if [ -n "$ARCHIVE_X86" ]; then
        ln -sf "$ROOT/external/TensorRT/x86/targets/x86_64-linux-gnu/lib/"*.so* \
               "$ROOT/.pixi/envs/arena/lib/"
        ln -sf "$ROOT/external/TensorRT/aarch64/targets/aarch64-linux-gnu/lib/"*.so* \
               "$ROOT/.pixi/aarch64-sysroot/lib/"
    fi

    # Build TRT engine files from ONNX models
    [ -n "$ARCHIVE_X86" ] && build_trt_engines

    docker build -f "$ROOT/src/arena/Dockerfile.circus"  -t colosseum/arena-circus  "$ROOT"
    docker build -f "$ROOT/src/arena/Dockerfile.booster" -t colosseum/arena-booster "$ROOT"
    echo "Setup complete."
}
main "$@"
```

### 1c. `pyproject.toml` additions

```toml
[tool.pixi.environments.arena-robot]
features = ["arena", "cross-compile"]
no-default-feature = true

[tool.pixi.feature.cross-compile.target.linux-64.dependencies]
gcc_linux-aarch64 = ">=14.3.0,<15"
gxx_linux-aarch64 = ">=14.3.0,<15"
cmake = "==4.3.0"

[tool.pixi.feature.cross-compile.tasks]
_configure-aarch64 = { cmd = "cmake -S $PIXI_PROJECT_ROOT -B $PIXI_PROJECT_ROOT/build/aarch64 -DCMAKE_PREFIX_PATH=$CONDA_PREFIX --toolchain $PIXI_PROJECT_ROOT/src/arena/cmake/aarch64-toolchain.cmake" }
_compile-aarch64   = { cmd = "cmake --build $PIXI_PROJECT_ROOT/build/aarch64 --parallel 8", depends-on = ["_configure-aarch64"] }
```

Add to existing `[tool.pixi.feature.arena.tasks]`:

```toml
setup           = { cmd = "bash $PIXI_PROJECT_ROOT/src/arena/scripts/setup.sh" }
configure-aarch64 = { cmd = "pixi run -e arena-robot _configure-aarch64" }
compile-aarch64   = { cmd = "pixi run -e arena-robot _compile-aarch64" }
sim-circus  = { cmd = "circus --scene $PIXI_PROJECT_ROOT/src/arena/sim/scene.yaml --framework $PIXI_PROJECT_ROOT/src/arena/sim/framework_circus.yaml" }
sim-booster = { cmd = "circus --scene $PIXI_PROJECT_ROOT/src/arena/sim/scene.yaml --framework $PIXI_PROJECT_ROOT/src/arena/sim/framework_booster.yaml" }
```

### 1d. Remaining arena files

#### `src/arena/cmake/aarch64-toolchain.cmake`

Copy verbatim from `/home/neverorfrog/code/spqr/spqrbooster2026/cmake/aarch64-toolchain.cmake`.
Sysroot path `$PIXI_PROJECT_ROOT/.pixi/aarch64-sysroot` is identical.

#### `src/arena/scripts/pixi_install_aarch64.sh`

Copy verbatim from `/home/neverorfrog/code/spqr/spqrbooster2026/scripts/pixi_install_aarch64.sh`.
Fully generic — no project-specific logic.

#### `src/arena/sim/scene.yaml`

```yaml
simulation_config: default
teams:
  red:
    number: 1
    players:
      - type: Booster-T1
        number: 1
        position: [0.0, 0.0, 0.68]
        orientation: [0.0, 0.0, 0.0]
```

#### `src/arena/sim/framework_circus.yaml` and `framework_booster.yaml`

```yaml
# framework_circus.yaml (Scenario A — CircusPortal)
image: colosseum/arena-circus
volumes: []

# framework_booster.yaml (Scenario B — DDS via booster-motion)
image: colosseum/arena-booster
volumes: []
```

No `<placeholder>` substitution needed — arena binary is volume-mounted from host at runtime.

#### `src/arena/Dockerfile.circus` (Scenario A — minimal)

```dockerfile
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y libstdc++6 && rm -rf /var/lib/apt/lists/*
# Arena binary volume-mounted from host /workspace for fast iteration
ENTRYPOINT ["/workspace/src/arena/scripts/docker_entrypoint_circus.sh"]
```

#### `src/arena/scripts/docker_entrypoint_circus.sh`

```bash
#!/usr/bin/env bash
set -e
exec /workspace/build/src/arena/main \
    --backend circus \
    --task t1-velocity-flat \
    --host "${SERVER_IP:-172.17.0.1}" \
    --port "${CIRCUS_PORT:-5555}" \
    --robot "${ROBOT_NAME:-T1}"
```

`SERVER_IP`, `CIRCUS_PORT`, `ROBOT_NAME` injected automatically by Circus as env vars.

#### `src/arena/Dockerfile.booster` (Scenario B — mirrors spqrbooster2026)

```dockerfile
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    libmsgpack-dev libasound2 libpulse0 libxcursor1 libxinerama1 libxi6 \
    libxrandr2 libxss1 libxxf86vm1 libdrm2 libgbm1 libwayland-egl1 \
    libwayland-client0 libwayland-cursor0 libxkbcommon0 libdecor-0-0 \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

COPY external/simbridge/msg/ /bridge_ws/msg/
COPY external/simbridge/src/ /bridge_ws/src/
COPY external/simbridge/tools/ /bridge_ws/tools/
COPY external/simbridge/CMakeLists.txt /bridge_ws/

RUN mv /bridge_ws/tools/booster_motion /opt/booster
RUN source /opt/ros/humble/setup.bash && \
    cmake -S /bridge_ws -B /tmp/build -DCMAKE_INSTALL_PREFIX=/opt/ros/humble && \
    cmake --build /tmp/build && rm -rf /tmp/build

ENV LD_LIBRARY_PATH=/opt/booster/lib:/opt/booster/lib-usr-local:/opt/booster/lib-x86_64-linux-gnu
ENV BOOSTER_ROOT=/opt/booster

ENTRYPOINT ["/workspace/src/arena/scripts/docker_entrypoint_booster.sh"]
```

#### `src/arena/scripts/docker_entrypoint_booster.sh`

```bash
#!/usr/bin/env bash
set -e
source /ros_entrypoint.sh
cd /opt/booster && ./booster-motion -mode sim -config configs/config_isaac.lua &
PIXI_SYSROOT="/workspace/.pixi/envs/arena/x86_64-conda-linux-gnu/sysroot"
"${PIXI_SYSROOT}/lib64/ld-linux-x86-64.so.2" \
    --library-path "${PIXI_SYSROOT}/lib64:/workspace/.pixi/envs/arena/lib" \
    /workspace/build/src/arena/main --backend booster --task t1-velocity-flat &
wait -n
```

### Developer flow

```bash
# One-time dev machine setup (downloads TRT, builds Docker images, creates sysroot)
pixi run -e arena setup

# x86 inference benchmark
pixi run -e arena compile
./build/src/arena/main --backend mujoco --task t1-velocity-flat --inference onnx
./build/src/arena/main --backend mujoco --task t1-velocity-flat --inference trt

# Sim2sim
pixi run -e arena sim-circus

# Robot deployment
pixi run compile-aarch64
bash src/arena/scripts/deploy.sh --ip 192.168.10.102
```

---

## Part 2: Policy Contract

### `policies/t1_velocity_flat.yaml`

Ground truth: fill observation list from `src/arena/src/tasks/T1VelocityFlat.cpp:build_observation()`.

```yaml
task: t1-velocity-flat
robot: booster-t1
policy_dt: 0.02
action_dim: 23

# Supported inference backends
inference_backends:
  onnx: models/t1-velocity-flat/t1-velocity-flat.onnx
  trt:  models/t1-velocity-flat/t1-velocity-flat.engine  # GPU-specific, built by setup.sh

observations:
  - name: projected_gravity
    dim: 3
  - name: base_ang_vel
    dim: 3
  - name: velocity_command
    dim: 3
  - name: joint_pos_rel
    dim: 23
  - name: joint_vel
    dim: 23
  - name: last_action
    dim: 23
  # fill remaining from T1VelocityFlat.cpp
```

---

## Part 3: Circus CLI Dependency Inversion

### Design intent

Both consuming projects migrate to `--scene`/`--framework`. Circus has no hardcoded project
knowledge. The GUI scene selector is bypassed when `--scene` is given (existing code at
`AppWindow.cpp:99` already handles this for the positional arg case).

spqrbooster2026 will also own its sim files and pass `--paths` if its framework config uses
`<placeholder>` substitution:

```toml
sim-1v1 = { cmd = "circus --scene $PIXI_PROJECT_ROOT/sim/1v1.yaml --framework $PIXI_PROJECT_ROOT/sim/framework_config.yaml --paths $PIXI_PROJECT_ROOT/sim/path_constants.yaml" }
```

### Files to modify (`/home/neverorfrog/code/spqr/circus/`)

| File | Change |
|------|--------|
| `include/AppWindow.h` | Add `frameworkConfigPath_` and `pathsConfigPath_` members |
| `src/AppWindow.cpp:39–42` | Parse `--scene`/`--framework`/`--paths` |
| `src/AppWindow.cpp:224` | Pass members to `startContainers()` |
| `include/RobotManager.h` | Extend `startContainers()` signature |
| `src/RobotManager.cpp:55` | Use passed paths; load `pathsRoot` lazily |

### `include/AppWindow.h`

```cpp
private:
    std::string frameworkConfigPath_ = spqr::frameworkConfigPath;
    std::string pathsConfigPath_     = spqr::pathsConfigPath;
```

### `src/AppWindow.cpp` — replace lines 39–42

```cpp
std::optional<std::string> scenePath;
for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if      (arg == "--scene"     && i + 1 < argc) scenePath            = argv[++i];
    else if (arg == "--framework" && i + 1 < argc) frameworkConfigPath_ = argv[++i];
    else if (arg == "--paths"     && i + 1 < argc) pathsConfigPath_     = argv[++i];
}
```

### `src/AppWindow.cpp:224`

```cpp
RobotManager::instance().startContainers(frameworkConfigPath_, pathsConfigPath_);
```

### `include/RobotManager.h`

```cpp
void startContainers(const std::string& fwkCfgPath  = spqr::frameworkConfigPath,
                     const std::string& pathsCfgPath = spqr::pathsConfigPath);
```

### `src/RobotManager.cpp` — updated `startContainers()`

```cpp
void RobotManager::startContainers(const std::string& fwkCfgPath,
                                   const std::string& pathsCfgPath) {
    startCommunicationServer(frameworkCommunicationPort);
    YAML::Node configRoot = loadYamlFile(fwkCfgPath);

    std::string image = tryString(configRoot["image"], "'image' must be a string");
    std::vector<std::string> binds;
    std::optional<YAML::Node> pathsRoot;

    for (const auto& v : configRoot["volumes"]) {
        std::string v2 = tryString(v, "Volume entry must be a string");
        if (v2.starts_with("<")) {
            if (!pathsRoot) pathsRoot = loadYamlFile(pathsCfgPath);
            std::string name = v2.substr(1, v2.find('>') - 1);
            if (!(*pathsRoot)[name])
                throw std::runtime_error("Entry missing in path_constants: " + name);
            v2.replace(0, v2.find('>') + 1,
                       tryString((*pathsRoot)[name], "path_constants values must be strings"));
        }
        binds.push_back(v2);
    }
    for (std::shared_ptr<Robot> r : robots_) {
        r->container = std::make_unique<Container>("CIRCUS_" + r->name + "_container");
        r->container->create(r, image, binds);
        r->container->start();
    }
}
```

---

## Part 4: Remove Python Deploy Dead Code

1. Delete `src/colosseum/utils/deploy/` (entire directory)
2. Remove `[tool.pixi.environments.deploy]` and `[tool.pixi.feature.deploy]` from `pyproject.toml`
3. Verify: `grep -r "colosseum.utils.deploy" src/`

---

## Verification

### Inference benchmark

```bash
pixi run -e arena compile
./build/src/arena/main --backend mujoco --task t1-velocity-flat --inference onnx
./build/src/arena/main --backend mujoco --task t1-velocity-flat --inference trt
```

### Circus CLI

```bash
circus --scene /path/to/src/arena/sim/scene.yaml \
       --framework /path/to/src/arena/sim/framework_circus.yaml
```

### Sim2sim end-to-end (Scenario A)

```bash
pixi run -e arena compile
pixi run -e arena sim-circus   # Circus spawns arena-circus container → T1 in Circus window
```

### aarch64 deploy

```bash
pixi run compile-aarch64
bash src/arena/scripts/deploy.sh --ip 192.168.10.102
# ssh booster@192.168.10.102; cd arena; ./run.sh run
```

---

## Implementation Steps

Each step is atomic and independently verifiable.

### Step 1 — Inference engine abstraction (colosseum/arena)

1. Create `src/arena/include/IInferenceEngine.h` with the abstract interface
2. Add `public IInferenceEngine` to `OnnxPolicy` in `src/arena/include/OnnxPolicy.h`
3. Create `src/arena/include/TrtEngine.h` and `src/arena/src/TrtEngine.cpp`
4. Add `inference_backend = "onnx"` field to `TaskConfig` in `src/arena/include/TaskConfig.h`
5. Refactor `src/arena/include/Policy.h`: replace `OnnxPolicy onnx` with `std::unique_ptr<IInferenceEngine> engine_`; add `make_engine()` factory; replace all `onnx.*` with `engine_->`
6. Add `--inference onnx|trt` parsing to `src/arena/src/main.cpp`; set `cfg.inference_backend`
7. Update `src/arena/CMakeLists.txt`: add conditional `find_library(TENSORRT_LIB nvinfer)` + `TrtEngine.cpp` + `WITH_TENSORRT` compile definition
8. **Verify**: `pixi run -e arena compile` succeeds; `--inference onnx` runs; `--inference trt` prints "TRT not built" if TRT unavailable

### Step 2 — Cross-compile buildchain (colosseum)

1. Copy `cmake/aarch64-toolchain.cmake` → `src/arena/cmake/aarch64-toolchain.cmake`
2. Copy `scripts/pixi_install_aarch64.sh` → `src/arena/scripts/pixi_install_aarch64.sh`
3. Add `arena-robot` env + `cross-compile` feature + tasks to `pyproject.toml`
4. **Verify**: `pixi run compile-aarch64` produces `build/aarch64/src/arena/main`; `file` confirms aarch64 ELF

### Step 3 — sim2sim files (colosseum/arena)

1. Create `src/arena/sim/scene.yaml`
2. Create `src/arena/sim/framework_circus.yaml`
3. Create `src/arena/sim/framework_booster.yaml`
4. Create `src/arena/Dockerfile.circus`
5. Create `src/arena/Dockerfile.booster`
6. Create `src/arena/scripts/docker_entrypoint_circus.sh` (chmod +x)
7. Create `src/arena/scripts/docker_entrypoint_booster.sh` (chmod +x)
8. Add `sim-circus` and `sim-booster` tasks to `pyproject.toml`

### Step 4 — setup.sh (colosseum/arena)

1. Create `src/arena/scripts/setup.sh` with GPU detection, TRT download, pixi install, sysroot creation, Docker image builds
2. Add `setup` pixi task to `pyproject.toml`
3. Move `scripts/deploy.sh`, `scripts/pack_channel.sh`, `scripts/run.sh` → `src/arena/scripts/`; update any path references inside those scripts
4. **Verify**: `pixi run -e arena setup` completes; `docker images | grep colosseum` shows both images

### Step 5 — Policy contract (colosseum)

1. Create `policies/` directory at repo root
2. Create `policies/t1_velocity_flat.yaml`: read `src/arena/src/tasks/T1VelocityFlat.cpp:build_observation()` to fill exact observation list and dims

### Step 6 — Circus CLI (circus repo)

1. Edit `include/AppWindow.h`: add `frameworkConfigPath_` and `pathsConfigPath_` members
2. Edit `src/AppWindow.cpp:39–42`: replace positional arg check with `--scene`/`--framework`/`--paths` loop
3. Edit `src/AppWindow.cpp:224`: pass members to `startContainers()`
4. Edit `include/RobotManager.h`: extend `startContainers()` signature
5. Edit `src/RobotManager.cpp:55`: update implementation (lazy pathsRoot load)
6. **Verify**: build circus; run `circus --scene src/arena/sim/scene.yaml --framework src/arena/sim/framework_circus.yaml`; confirm Circus loads the scene without file picker

### Step 7 — Remove Python deploy dead code (colosseum)

1. Delete `src/colosseum/utils/deploy/` (3 files)
2. Remove `[tool.pixi.environments.deploy]` and `[tool.pixi.feature.deploy]` sections from `pyproject.toml`
3. Run `grep -r "colosseum.utils.deploy" src/` — confirm zero results
4. **Verify**: `pixi install -e train` still resolves cleanly
