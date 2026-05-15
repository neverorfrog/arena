# ================== PROJECT SETUP ===================
cmake_minimum_required(VERSION 3.22)

project(arena VERSION 0.1 LANGUAGES CXX)

# Use the conda sysroot glibc at runtime so the binary doesn't fall back to
# the host system glibc.
if(DEFINED ENV{CONDA_PREFIX} AND NOT CMAKE_CROSSCOMPILING)
  set(_sysroot "$ENV{CONDA_PREFIX}/x86_64-conda-linux-gnu/sysroot")
  if(EXISTS "${_sysroot}/lib64/ld-linux-x86-64.so.2")
    set(CMAKE_BUILD_RPATH "${_sysroot}/lib64;$ENV{CONDA_PREFIX}/lib")
    string(
            APPEND CMAKE_EXE_LINKER_FLAGS
            " -Wl,-dynamic-linker,${_sysroot}/lib64/ld-linux-x86-64.so.2"
        )
  endif()
endif()

# When cross-compiling, use the sysroot's dynamic linker on the robot so
# we match the sysroot's glibc (not the robot's older system glibc).
# The sysroot is transferred to $ROBOT_PATH/.pixi/aarch64-sysroot by
# setup_robot.sh.
if(CMAKE_CROSSCOMPILING)
  set(_robot_sysroot "/home/booster/spqr/arena/.pixi/aarch64-sysroot")
  set(CMAKE_BUILD_RPATH "${_robot_sysroot}/lib64;${_robot_sysroot}/lib")
  string(APPEND CMAKE_EXE_LINKER_FLAGS
    " -Wl,-dynamic-linker,${_robot_sysroot}/lib/ld-linux-aarch64.so.1"
  )
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_definitions(PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

# ================ Dependencies ======================
# Prefer -pthread over -lpthreads (cross-compilation can't detect
# pthread-in-libc because CMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY
# prevents linking; the aarch64 sysroot only has libpthread, not libpthreads).
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(booster_robotics_sdk REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(msgpack-cxx REQUIRED)

# MuJoCo backend (optional -- enables --backend mujoco at runtime)
find_package(mujoco QUIET)
find_package(glfw3  QUIET)
if(mujoco_FOUND AND glfw3_FOUND)
  message(STATUS "MuJoCo ${mujoco_VERSION} found -- MuJoCo backend enabled")
  set(MUJOCO ON)
else()
  message(STATUS "MuJoCo or GLFW not found -- MuJoCo backend disabled")
  set(MUJOCO OFF)
endif()

# TensorRT (optional -- enables --inference trt at runtime)
find_library(TENSORRT_LIB nvinfer)
if(TENSORRT_LIB)
  # In aarch64 cross-compile, find_package(CUDAToolkit) reports include dirs
  # under the (x86_64) conda env which don't exist on disk, and bakes them
  # into the CUDA::cudart imported target. Locate cudart and the headers in
  # the sysroot directly instead.
  if(CMAKE_CROSSCOMPILING)
    find_library(CUDA_CUDART cudart
      HINTS ${CMAKE_SYSROOT}/lib
            ${CMAKE_SYSROOT}/targets/aarch64-linux/lib
      NO_DEFAULT_PATH
      REQUIRED
    )
    find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
      HINTS ${CMAKE_SYSROOT}/targets/aarch64-linux/include
            ${CMAKE_SYSROOT}/targets/sbsa-linux/include
            ${CONDA_PREFIX}/targets/x86_64-linux/include
      NO_DEFAULT_PATH
      REQUIRED
    )
    # NvInfer.h is bundled in external/TensorRT/, outside the sysroot.
    # NO_CMAKE_FIND_ROOT_PATH bypasses the sysroot-only restriction.
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
      HINTS ${CMAKE_SOURCE_DIR}/external/TensorRT/aarch64/include
            ${CMAKE_SOURCE_DIR}/external/TensorRT/x86/include
            ${CONDA_PREFIX}/include
      NO_CMAKE_FIND_ROOT_PATH
      REQUIRED
    )
  else()
    find_library(CUDA_CUDART cudart
      HINTS $ENV{CONDA_PREFIX}/lib
    )
    find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
      HINTS $ENV{CONDA_PREFIX}/targets/x86_64-linux/include
    )
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
      HINTS ${CMAKE_SOURCE_DIR}/external/TensorRT/x86/include
    )
    if(NOT CUDA_CUDART OR NOT CUDA_INCLUDE_DIR OR NOT TENSORRT_INCLUDE_DIR)
      message(STATUS "CUDA or TensorRT headers not found -- TRT backend disabled")
      set(TENSORRT_LIB "" CACHE INTERNAL "" FORCE)
    endif()
  endif()
  if(TENSORRT_LIB)
    message(STATUS "TensorRT found -- TRT backend enabled")
    message(STATUS "  nvinfer:    ${TENSORRT_LIB}")
    message(STATUS "  NvInfer.h:  ${TENSORRT_INCLUDE_DIR}")
    message(STATUS "  cudart:     ${CUDA_CUDART}")
    message(STATUS "  CUDA:       ${CUDA_INCLUDE_DIR}")
    set(WITH_TRT ON)
  endif()
endif()

# ====================================================

# =================== Library ====================

file(
    GLOB_RECURSE SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER SOURCES EXCLUDE REGEX ".*/main\\.cpp$")
list(FILTER SOURCES EXCLUDE REGEX ".*/benchmark\\.cpp$")
list(FILTER SOURCES EXCLUDE REGEX ".*/arena_start\\.cpp$")
list(FILTER SOURCES EXCLUDE REGEX ".*/arena_stop\\.cpp$")
list(FILTER SOURCES EXCLUDE REGEX ".*/dds_debug\\.cpp$")
list(FILTER SOURCES EXCLUDE REGEX ".*/tests/.*")

if(NOT MUJOCO)
  list(FILTER SOURCES EXCLUDE REGEX ".*/MujocoPortal\\.cpp$")
endif()
if(NOT WITH_TRT)
  list(FILTER SOURCES EXCLUDE REGEX ".*/TrtInferenceEngine\\.cpp$")
endif()

add_library(arena OBJECT ${SOURCES})
target_include_directories(
    arena
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(
    arena
    PUBLIC
        Eigen3::Eigen
        onnxruntime::onnxruntime
        msgpack-cxx
        booster::booster_robotics_sdk
        -Wl,--no-as-needed
        Threads::Threads
        rt
        -Wl,--as-needed
)
if(MUJOCO)
  target_link_libraries(arena PUBLIC mujoco::mujoco glfw)
  target_compile_definitions(arena PUBLIC MUJOCO)
endif()
if(WITH_TRT)
  target_include_directories(arena SYSTEM PUBLIC ${CUDA_INCLUDE_DIR} ${TENSORRT_INCLUDE_DIR})
  target_compile_definitions(arena PUBLIC WITH_TENSORRT)
  target_link_libraries(arena PUBLIC ${TENSORRT_LIB} ${CUDA_CUDART})
  if(CMAKE_CROSSCOMPILING)
    # The aarch64 conda CUDA package omits targets/sbsa-linux/include/crt/,
    # which cuda_runtime_api.h pulls in via #include "crt/host_defines.h".
    # Borrow the host's x86_64 conda env crt/ headers — they're host-side
    # macro definitions, not architecture-specific.
    if(EXISTS "$ENV{CONDA_PREFIX}/targets/x86_64-linux/include/crt")
      target_include_directories(arena SYSTEM PUBLIC
        "$ENV{CONDA_PREFIX}/targets/x86_64-linux/include"
      )
    endif()
    # libnvinfer.so references Tegra-only libs (libcudla, libnvdla) that
    # exist on the robot but not in the cross-compile sysroot. Defer those
    # symbols to runtime resolution.
    target_link_options(arena PUBLIC
      $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-Wl,--allow-shlib-undefined>
    )
  endif()
endif()

# ====================================================

# ============================== TESTS ==============================================
find_package(GTest QUIET)

if(GTest_FOUND)
  message(STATUS "GTest found - building tests for ${PROJECT_NAME}")

  file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp")
  file(GLOB TEST_FIXTURES "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures.cpp")

  if(TEST_FIXTURES)
    add_library(${PROJECT_NAME}_test_common STATIC ${TEST_FIXTURES})
    target_include_directories(
            ${PROJECT_NAME}_test_common
            PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tests
        )
    target_link_libraries(
            ${PROJECT_NAME}_test_common
            PUBLIC GTest::gtest arena
        )
    set(TEST_COMMON_LIB ${PROJECT_NAME}_test_common)
  endif()

  file(
        GLOB_RECURSE TESTABLE_SOURCES
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/engines/OnnxInferenceEngine.cpp"
    )

  add_executable(
        ${PROJECT_NAME}_tests
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/main.cpp
        ${TEST_SOURCES}
        ${TESTABLE_SOURCES}
    )

  target_include_directories(
        ${PROJECT_NAME}_tests
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

  target_link_libraries(
        ${PROJECT_NAME}_tests
        PRIVATE GTest::gtest Eigen3::Eigen onnxruntime::onnxruntime ${TEST_COMMON_LIB}
    )

  enable_testing()
  add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)

  install(TARGETS ${PROJECT_NAME}_tests RUNTIME DESTINATION bin)
else()
  message(STATUS "GTest not found - skipping tests for ${PROJECT_NAME}")
endif()
# ===================================================================================

# ================== Installation ====================
install(
    TARGETS arena
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
)

# Models installed from external/colosseum/models/ at runtime (MODELS_DIR env var)
# ====================================================

# ====================== Main ======================
add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_include_directories(
    main
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(main PRIVATE arena)
install(TARGETS main RUNTIME DESTINATION bin)
# ====================================================

# ====================== Benchmark ======================
add_executable(benchmark ${CMAKE_CURRENT_SOURCE_DIR}/src/benchmark.cpp)
target_include_directories(
    benchmark
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(benchmark PRIVATE arena)
install(TARGETS benchmark RUNTIME DESTINATION bin)
# ====================================================

# ====================== Arena Start/Stop Daemons ======================
# DDS-triggered start/stop daemons, adapted from spqrbooster2026.
# These subscribe to rt/remote_controller_state and spawn or kill
# the arena_main process in response to joystick button combos.

# arena_start — LB+RB+A to start the arena systemd service
add_executable(arena_start ${CMAKE_CURRENT_SOURCE_DIR}/src/arena_start.cpp)
target_include_directories(
    arena_start
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(arena_start PRIVATE booster::booster_robotics_sdk Threads::Threads)
install(TARGETS arena_start RUNTIME DESTINATION bin)

# arena_stop — LB+RB+X to SIGTERM arena_main
add_executable(arena_stop ${CMAKE_CURRENT_SOURCE_DIR}/src/arena_stop.cpp)
target_include_directories(
    arena_stop
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(arena_stop PRIVATE booster::booster_robotics_sdk Threads::Threads)
install(TARGETS arena_stop RUNTIME DESTINATION bin)

# dds_debug — DDS diagnostic tool (uses RobotPortal + create_input_source like main)
add_executable(dds_debug ${CMAKE_CURRENT_SOURCE_DIR}/src/dds_debug.cpp)
target_include_directories(
    dds_debug
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dds_debug PRIVATE arena)
install(TARGETS dds_debug RUNTIME DESTINATION bin)
# ====================================================
