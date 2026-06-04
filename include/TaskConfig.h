#pragma once
#include "RobotConfig.h"
#include <string>

// Task-level deployment configuration.
//
// Robot hardware specs (gains, limits, joint names, XML path) live in `robot`
// and are shared across any task that uses the same robot platform.
// Task-specific fields are: model checkpoint, timing, action scale, scene XML.
struct TaskConfig {
    static constexpr int NUM_JOINTS  = 23;  // total robot joints
    static constexpr int NUM_ACTIONS = 21;  // policy output dim (head joints excluded)
    using Robot = RobotConfig<NUM_JOINTS>;

    std::string task_name;
    std::string model_name;       // Named model to load (empty = default)
    std::string model_path;       // Path to ONNX checkpoint (resolved by ModelRegistry)
    float       policy_dt   = 0.02f;  // Policy step period (s) — 50 Hz

    // Per-action scale (21 elements, one per policy output).
    // Decoded: target[joint] = net_out[a] * action_scale[a] + default_joint_pos[joint]
    // where joint = action_to_joint_idx[a].
    // Must match training: typically 0.25 (Python PolicyConfig.action_scale).
    std::array<float, NUM_ACTIONS> action_scale{};

    // Maps action index (0..NUM_ACTIONS-1) to hardware joint index (0..NUM_JOINTS-1).
    // Head joints are excluded; their targets come from post_decode_action.
    std::array<int, NUM_ACTIONS> action_to_joint_idx{};

    // Inference backend: "onnx" (default) or "trt" (TensorRT).
    // Threaded through Policy → make_engine() factory.
    std::string inference_backend = "onnx";

    Robot robot;  // Hardware specs: gains, limits, joint names, XML path

    // Path to the scene MJCF loaded by MujocoPortal.
    // The robot XML is loaded separately from robot.mjcf_path.
    std::string scene_mjcf_path;

    bool debug = false;  // Enable per-step diagnostic output
};
