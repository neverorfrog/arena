#pragma once
#include "RobotConfig.h"
#include <algorithm>
#include <stdexcept>
#include <string>

// Runtime robot state with precomputed joint-order remapping.
//
// Joint arrays are stored in REAL HARDWARE order (same as RobotConfig::joint_names
// and as received from DDS LowState). The policy observation requires joints in
// MuJoCo simulation order (RobotConfig::sim_joint_names). The two index arrays
// below enable efficient, zero-copy remapping at every inference step.
//
// Index semantics:
//   real2sim[i] — index into the hardware-order arrays that gives the joint
//                 whose sim name is sim_joint_names[i].
//                 Use when building the observation: obs_joint[i] = joint_pos[real2sim[i]]
//
//   sim2real[i] — index into the sim-order arrays that gives the joint
//                 whose hardware name is joint_names[i].
//                 Use when decoding actions: hardware_target[i] = action[sim2real[i]]
//
// When joint_names == sim_joint_names (same order), both arrays are the identity
// permutation: real2sim[i] == i and sim2real[i] == i. This is the current T1
// configuration, so existing behavior is preserved with zero extra cost.
//
// Construction throws std::runtime_error if either name array contains a name
// not present in the other — catches configuration errors at startup.
template<int N>
struct RobotData {

    // Joint state — all in REAL HARDWARE order.
    std::array<float, N> joint_pos{};
    std::array<float, N> joint_vel{};
    std::array<float, N> feedback_torque{};

    // Base/IMU state (body frame unless noted).
    float root_pos_w[3]{};           // world position (m)
    float root_quat_w[4]{1,0,0,0};  // world orientation (w,x,y,z)
    float root_lin_vel_b[3]{};       // linear velocity in body frame (m/s)
    float root_ang_vel_b[3]{};       // angular velocity in body frame (rad/s)
    float projected_gravity_b[3]{};  // gravity vector in body frame (m/s²)

    // Precomputed index maps (see class doc above).
    std::array<int, N> real2sim{};
    std::array<int, N> sim2real{};

    explicit RobotData(const RobotConfig<N>& cfg) {
        // real2sim[i]: which hardware index feeds sim slot i
        for (int i = 0; i < N; i++) {
            const std::string& sim_name = cfg.sim_joint_names[i];
            auto it = std::find(cfg.joint_names.begin(), cfg.joint_names.end(), sim_name);
            if (it == cfg.joint_names.end())
                throw std::runtime_error(
                    "RobotData: sim joint '" + sim_name
                    + "' not found in joint_names for robot '" + cfg.name + "'");
            real2sim[i] = static_cast<int>(std::distance(cfg.joint_names.begin(), it));
        }

        // sim2real[i]: which sim index feeds hardware slot i
        for (int i = 0; i < N; i++) {
            const std::string& real_name = cfg.joint_names[i];
            auto it = std::find(cfg.sim_joint_names.begin(), cfg.sim_joint_names.end(), real_name);
            if (it == cfg.sim_joint_names.end())
                throw std::runtime_error(
                    "RobotData: hardware joint '" + real_name
                    + "' not found in sim_joint_names for robot '" + cfg.name + "'");
            sim2real[i] = static_cast<int>(std::distance(cfg.sim_joint_names.begin(), it));
        }
    }
};
