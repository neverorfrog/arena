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

ensure_pip() {
    if command -v python3 >/dev/null 2>&1 && python3 -m pip --version >/dev/null 2>&1; then
        return
    fi
    echo "pip not found, installing via apt..."
    sudo apt-get update
    sudo apt-get install -y python3-pip
}

TRT_DIR="$ROOT/external/TensorRT"

download_tensorrt() {
    local archive="$1"
    ensure_pip
    pixi global install gdown
    declare -A FILE_MAP=(
        ["TensorRT-10.3.0.26.Linux.x86_64-gnu.cuda-12.5.tar.gz"]="1Tfah5ssWglzavaSPXwoTpUI5SKW7Twxx"
        ["TensorRT-10.13.2.6.Linux.x86_64-gnu.cuda-13.0.tar.gz"]="1IfYm0AWyEQ5-5LeUhivrlAwN83MO4nti"
        ["TensorRT-10.3.0.26.l4t.aarch64-gnu.cuda-12.6.tar.gz"]="17gxgbAxjzBpL2OSMr4Ne9jMeBBgsTlbU"
    )
    FILE_ID="${FILE_MAP[$archive]}"
    if [[ -z "$FILE_ID" ]]; then
        echo "File $archive not found in FILE_MAP. Update FILE_MAP accordingly inside setup.sh"
        exit 1
    fi
    local output="$TRT_DIR/$archive"
    if [ -f "$output" ]; then
        echo "Already downloaded"
        return
    fi
    gdown "https://drive.google.com/uc?id=$FILE_ID" -O "$output"
}

extract() {
    local archive="$1"
    local rename="$2"
    local path="$TRT_DIR/$archive"
    if [ ! -f "$path" ]; then
        echo "Archive not found: $path"
        exit 1
    fi
    tar -xzf "$path" -C "$TRT_DIR"
    local extracted
    extracted=$(tar -tf "$path" | head -n1 | cut -d/ -f1)
    mv "$TRT_DIR/$extracted" "$TRT_DIR/$rename"
}

build_trt_engines() {
    for model_dir in "$ROOT"/external/colosseum/models/*/; do
        local raw_onnx="${model_dir}raw/$(basename "$model_dir").onnx"
        local engine="${model_dir}$(basename "$model_dir")/$(basename "$model_dir").engine"
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
        [ -d "$ROOT/external/TensorRT/x86" ]    || { download_tensorrt "$ARCHIVE_X86";    extract "$ARCHIVE_X86" "x86"; }
        [ -d "$ROOT/external/TensorRT/aarch64" ] || { download_tensorrt "$ARCHIVE_AARCH64"; extract "$ARCHIVE_AARCH64" "aarch64"; }
        rm -f "$ROOT/external/TensorRT"/*.tar.gz
    fi

    pixi install
    cd "$ROOT" && pixi_install_aarch64 robot

    # Symlink TRT libs into pixi envs
    if [ -n "$ARCHIVE_X86" ]; then
        ln -sf "$ROOT/external/TensorRT/x86/targets/x86_64-linux-gnu/lib/"*.so* \
               "$ROOT/.pixi/envs/default/lib/"
        ln -sf "$ROOT/external/TensorRT/aarch64/targets/aarch64-linux-gnu/lib/"*.so* \
               "$ROOT/.pixi/aarch64-sysroot/lib/"
    fi

    [ -n "$ARCHIVE_X86" ] && build_trt_engines

    docker build -t arena/arena-booster "$ROOT"
    echo "Setup complete."
}

main "$@"
