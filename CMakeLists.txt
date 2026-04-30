# ================== PROJECT SETUP ===================
cmake_minimum_required(VERSION 3.22)

project(arena VERSION 0.1 LANGUAGES CXX)

# Use the conda sysroot glibc at runtime so the binary doesn't fall back to
# the host system glibc.
if(DEFINED ENV{CONDA_PREFIX} AND NOT CMAKE_CROSSCOMPILING)
  set(_sysroot "$ENV{CONDA_PREFIX}/x86_64-conda-linux-gnu/sysroot")
  if(EXISTS "${_sysroot}/lib64/ld-linux-x86-64.so.2")
    set(CMAKE_BUILD_RPATH "${_sysroot}/lib64;$ENV{CONDA_PREFIX}/lib")
    string(APPEND CMAKE_EXE_LINKER_FLAGS
            " -Wl,-dynamic-linker,${_sysroot}/lib64/ld-linux-x86-64.so.2")
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_definitions(PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

# ================ Dependencies ======================
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
  message(STATUS "TensorRT found -- TRT backend enabled")
  set(WITH_TRT ON)
endif()

# ====================================================

# =================== Library ====================

file(
    GLOB_RECURSE SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER SOURCES EXCLUDE REGEX ".*/main\\.cpp$")
list(FILTER SOURCES EXCLUDE REGEX ".*/tests/.*")

if(NOT MUJOCO)
  list(FILTER SOURCES EXCLUDE REGEX ".*/MujocoPortal\\.cpp$")
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
  target_compile_definitions(arena PRIVATE WITH_TENSORRT)
  target_link_libraries(arena PUBLIC ${TENSORRT_LIB})
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
