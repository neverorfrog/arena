#pragma once
#include <array>
#include <string>
#include <vector>

// Safe startup pose configuration for the real robot (RobotPortal only).
//
// Before switching to policy control, the robot holds this pose with lower
// PD gains for `duration_s` seconds. This avoids aggressive initial commands
// when the robot may not be in the expected standing pose.
template<int N>
struct PrepareStateConfig {
    std::array<float, N> joint_pos{};   // safe starting pose (rad)
    std::array<float, N> stiffness{};   // reduced Kp during preparation
    std::array<float, N> damping{};     // Kd during preparation
    float                duration_s = 5.0f;
};

// Robot hardware specification. Task-agnostic: does not know about observations,
// rewards, or policy checkpoints.
//
// N is the number of controlled joints (compile-time constant).
// The current T1 23-DOF robot uses N=23.
//
// Joint orderings:
//   joint_names     — real hardware order (DDS LowState motor_state_serial)
//   sim_joint_names — MuJoCo compiled order (alphabetically sorted by MuJoCo)
//
// When both arrays contain the same names (just possibly in different order),
// RobotData computes the bidirectional remapping once at initialization.
// If the arrays are identical the mapping is the identity — zero overhead.
//
// All gain/limit arrays are indexed in HARDWARE order (same as joint_names).
template<int N>
struct RobotConfig {
    static constexpr int NUM_JOINTS = N;

    std::string name;  // human-readable identifier, e.g. "Booster_T1_23DOF"

    // Ordered joint name lists — must be the same set, possibly permuted.
    std::array<std::string, N> joint_names;      // real hardware order
    std::array<std::string, N> sim_joint_names;  // MuJoCo compiled order

    // PD gains (MUST match training). Indexed in hardware order.
    std::array<float, N> joint_stiffness;  // Kp (N·m/rad)
    std::array<float, N> joint_damping;    // Kd (N·m·s/rad)

    // Reflected inertia added to each joint's DoF in MuJoCo.
    // MujocoPortal sets mj_model_->dof_armature from this array.
    std::array<float, N> joint_armature{};

    // Coulomb friction loss per joint (N·m). MujocoPortal sets
    // mj_model_->dof_frictionloss from this array. Default 0 = no friction.
    std::array<float, N> joint_frictionloss{};

    // Standing pose (rad). Used as the reference for joint_pos_rel observations
    // and added back when decoding policy actions.
    std::array<float, N> default_joint_pos{};

    // Peak torques per joint (N·m). Used to clamp actuator output.
    std::array<float, N> effort_limit{};

    // Mechanically coupled joint index pairs (e.g. crank mechanisms).
    // If non-empty, portals replicate commands from the primary joint.
    std::vector<int> parallel_joint_indices;

    // Path to the robot MJCF/XML — loaded by MujocoPortal.
    std::string mjcf_path;

    // Safe startup configuration — consumed only by RobotPortal.
    PrepareStateConfig<N> prepare_state;
};
