#pragma once
#include "RobotConfig.h"
#include <string>

// Task-level deployment configuration.
//
// Robot hardware specs (gains, limits, joint names, XML path) live in `robot`
// and are shared across any task that uses the same robot platform.
// Task-specific fields are: model checkpoint, timing, action scale, scene XML.
struct TaskConfig {
    static constexpr int NUM_JOINTS = 23;
    using Robot = RobotConfig<NUM_JOINTS>;

    std::string task_name;
    std::string model_path;       // Path to ONNX checkpoint (resolved by ModelRegistry)
    float       policy_dt   = 0.02f;  // Policy step period (s) — 50 Hz

    // Uniform action scale applied to all joints.
    // Decoded action: target[i] = net_out[i] * action_scale + robot.default_joint_pos[i]
    // Must match training: typically 0.25 (Python PolicyConfig.action_scale).
    float action_scale = 0.25f;

    // Inference backend: "onnx" (default) or "trt" (TensorRT).
    // Threaded through Policy → make_engine() factory.
    std::string inference_backend = "onnx";

    Robot robot;  // Hardware specs: gains, limits, joint names, XML path

    // Path to the scene MJCF loaded by MujocoPortal.
    // The robot XML is loaded separately from robot.mjcf_path.
    std::string scene_mjcf_path;
};
