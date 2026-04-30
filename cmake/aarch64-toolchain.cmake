set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
# DO not execute compiled tests (that would fail since we are in x86)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Wipe pixi flags that are invalid for the aarch64
foreach(
    _env_var
    CFLAGS
    CXXFLAGS
    CPPFLAGS
    LDFLAGS
    LDFLAGS_LD
)
    if(DEFINED ENV{${_env_var}})
        unset(ENV{${_env_var}})
    endif()
endforeach()

set(CMAKE_C_COMPILER aarch64-conda-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-conda-linux-gnu-g++)

set(CMAKE_C_FLAGS "-Wno-psabi")
set(CMAKE_CXX_FLAGS "-Wno-psabi")

# This is meant to be created by setup.sh
set(_AARCH64_SYSROOT $ENV{PIXI_PROJECT_ROOT}/.pixi/aarch64-sysroot)
set(CMAKE_SYSROOT ${_AARCH64_SYSROOT})

set(CMAKE_FIND_ROOT_PATH ${_AARCH64_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
