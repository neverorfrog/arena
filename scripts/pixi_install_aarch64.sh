#!/bin/bash

# This is just an hack to overcome pixi's limitations.
# Apparently, there's no way to download aarch64 deps in an x86 system
# I found an hack, hoping that they will address this issue at some point: https://github.com/prefix-dev/pixi/issues/5918
# EDIT: apparently I WILL address the issue lol

pixi_install_aarch64() {
    local env_name="$1"
    local root_dir
    root_dir="$(pwd)"

    local dest="$root_dir/.pixi/aarch64-sysroot"
    local scratch="/tmp/pixi-aarch64-pack-$$"
    local conda_tmp_dir="/tmp/pixi-conda-tmp-$$"

    mkdir -p "$dest"

    rm -rf "$scratch"
    mkdir -p "$scratch"

    pixi exec pixi-pack --environment default --platform linux-aarch64 pixi.toml

    tar -xf environment.tar -C "$scratch"
    rm -f environment.tar

    for channel_dir in "$scratch"/channel/linux-aarch64 "$scratch"/channel/noarch; do

        for f in "$channel_dir"/*.conda; do

            local basename_f
            basename_f=$(basename "$f")

            # It's not clear why, but the conda archive also contains x86 executables, useless for crosscompiling
            case "$basename_f" in
                gcc_impl_*|gcc_*|gxx_*|binutils_*|cmake-*|clang-*|cuda-nvcc-*)
                    continue
                    ;;
            esac

            rm -rf "$conda_tmp_dir"
            mkdir -p "$conda_tmp_dir"
            unzip -q "$f" -d "$conda_tmp_dir"
            # extract only headers, libraries and cmake/pkgconfig data. We don't need binaries
            tar -xf "$conda_tmp_dir"/pkg-*.tar.zst -C "$dest/" include lib share targets 2>/dev/null || true
            # sysroot stuff
            tar -xf "$conda_tmp_dir"/pkg-*.tar.zst --strip-components=2 -C "$dest/" aarch64-conda-linux-gnu/sysroot/ 2>/dev/null || true
            rm -rf "$conda_tmp_dir"
        done

        for f in "$channel_dir"/*.tar.bz2; do
            [ -f "$f" ] || continue

            local basename_f
            basename_f=$(basename "$f")
            case "$basename_f" in
                gcc_impl_*|gcc_*|gxx_*|binutils_*|cmake-*|clang-*|cuda-nvcc-*)
                    continue
                    ;;
            esac

            tar -xjf "$f" -C "$dest/" include lib share targets 2>/dev/null || true
            tar -xjf "$f" --strip-components=2 -C "$dest/" aarch64-conda-linux-gnu/sysroot/ 2>/dev/null || true
        done
    done


    # If the robot has a jetson, the cuda sysroot should be under this directory. Please do verify it
    if [ -d "$dest/targets/sbsa-linux" ] && [ ! -d "$dest/targets/aarch64-linux" ]; then
        ln -sf sbsa-linux "$dest/targets/aarch64-linux"
    fi

    # Again, I'm trying to replicate what's inside the robot. This should be a standard for every linux filesystem
    # but we need to verify this.
    if [ -f "$dest/lib64/ld-linux-aarch64.so.1" ] && [ ! -f "$dest/lib/ld-linux-aarch64.so.1" ]; then
        mkdir -p "$dest/lib"
        ln -sf ../lib64/ld-linux-aarch64.so.1 "$dest/lib/ld-linux-aarch64.so.1"
    fi

    rm -rf "$scratch"

    cd "$root_dir" || exit 1
    echo "[aarch64] Environment '$env_name' ready."
}
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    pixi_install_aarch64 "$@"
fi
