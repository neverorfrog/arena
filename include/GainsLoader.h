#pragma once
#include "ModelConfig.h"
#include "RobotConfig.h"

#include <array>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

// Loads per-joint deploy gains from the gains.yaml that ships next to a model's
// ONNX, and applies them to a RobotConfig / action_scale by joint name.
//
// This replaces the hardcoded PD gains, armature, friction, effort limit, default
// pose and action scale in each task's make_config(): those values are part of the
// policy's training contract, so they travel with the checkpoint instead.

// Resolve gains.yaml as the sibling of the resolved ONNX path. ModelRegistry::resolve
// returns the canonical (symlink-resolved) ONNX, so its parent is the version dir.
inline std::filesystem::path gains_path_for_model(const std::string& model_path) {
    return std::filesystem::path(model_path).parent_path() / "gains.yaml";
}

// Fill a RobotConfig's per-joint arrays from gains, matched by hardware joint name.
template <int N>
void apply_gains(const GainsMap& gains, RobotConfig<N>& robot) {
    for (int i = 0; i < N; ++i) {
        auto it = gains.find(robot.joint_names[i]);
        if (it == gains.end())
            throw std::runtime_error("gains.yaml missing joint: " + robot.joint_names[i]);
        const JointGains& g = it->second;
        robot.joint_stiffness[i]    = g.kp;
        robot.joint_damping[i]      = g.kd;
        robot.joint_armature[i]     = g.armature;
        robot.joint_frictionloss[i] = g.frictionloss;
        robot.default_joint_pos[i]  = g.default_pos;
        robot.effort_limit[i]       = g.effort_limit;
    }
}

// Build the per-action scale vector: action_scale[a] = gains[joint(a)].action_scale.
template <int N>
std::vector<float> build_action_scale(const GainsMap& gains,
                                      const std::array<std::string, N>& joint_names,
                                      const std::vector<int>& action_to_joint_idx) {
    std::vector<float> scale(action_to_joint_idx.size());
    for (size_t a = 0; a < action_to_joint_idx.size(); ++a) {
        const std::string& joint = joint_names[action_to_joint_idx[a]];
        auto it = gains.find(joint);
        if (it == gains.end())
            throw std::runtime_error("gains.yaml missing joint: " + joint);
        scale[a] = it->second.action_scale;
    }
    return scale;
}
