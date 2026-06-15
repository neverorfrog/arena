#pragma once
#include "RobotConfig.h"
#include <string>
#include <vector>

// Task-level deployment configuration.
//
// Robot hardware specs (gains, limits, joint names, XML path) live in `robot`
// and are shared across any task that uses the same robot platform.
// Task-specific fields are: model checkpoint, timing, action scale, scene XML.
struct TaskConfig {
    static constexpr int NUM_JOINTS  = 23;  // total robot joints (hardware)
    using Robot = RobotConfig<NUM_JOINTS>;

    std::string task_name;
    std::string model_name;       // Named model to load (empty = default)
    std::string model_path;       // Path to ONNX checkpoint (resolved by ModelRegistry)
    float       policy_dt   = 0.02f;  // Policy step period (s) — 50 Hz

    // Per-action scale, one per policy output. Size must equal the skill's
    // policy output dimension (engine output_dim).
    // Decoded: target[joint] = net_out[a] * action_scale[a] + default_joint_pos[joint]
    // where joint = action_to_joint_idx[a].
    std::vector<float> action_scale;

    // Maps action index → hardware joint index (0..NUM_JOINTS-1). Same size as
    // action_scale. Joints not listed are held at their default pose by the
    // merged target buffer.
    std::vector<int> action_to_joint_idx;

    // Inference backend: "onnx" (default) or "trt" (TensorRT).
    // Threaded through Policy → make_engine() factory.
    std::string inference_backend = "onnx";

    Robot robot;  // Hardware specs: gains, limits, joint names, XML path

    // Path to the scene MJCF loaded by MujocoPortal.
    // The robot XML is loaded separately from robot.mjcf_path.
    std::string scene_mjcf_path;

    bool debug = false;  // Enable per-step diagnostic output
};
