#include "skills/LocomotionSkill12.h"

#include "ObservationSpec.h"

#include <algorithm>
#include <cmath>

// Observation layout contract for the 12-DOF legs-only policy. Mirrors the
// training colosseum observation_cfg.py term order exactly.
namespace {
struct Velocity12ObservationSpec : ObservationSpec {
    std::vector<Component> components() const override {
        return {
            {"proj_gravity",  3},
            {"base_ang_vel",  3},
            {"vel_command",   3},
            {"gait_phase",    2},
            {"joint_pos",    12},
            {"joint_vel",    12},
            {"last_action",  12},
        };
    }
};
}  // namespace

LocomotionSkill12::LocomotionSkill12(const TaskConfig& cfg)
    : LocomotionSkill(cfg) {
    // booster_gym locomotion gait frequency sampled from [1.0, 2.0] Hz; deploy
    // uses the fixed midpoint (1.5 Hz). Gate on |v| > 0.05 (twist threshold).
    gait_phase_ = GaitPhaseCommand(GaitPhaseCommandConfig{1.5f, 1.5f, 1.0f, 0.05f});
}

void LocomotionSkill12::build_observation(const RobotState& state) {
    static const Velocity12ObservationSpec spec;
    observation_.clear();
    last_yaw_ = state.rpy[2];

    // 1. Projected gravity (3)
    observation_.push_back(state.projected_gravity[0]);
    observation_.push_back(state.projected_gravity[1]);
    observation_.push_back(state.projected_gravity[2]);

    // 2. Base angular velocity (3)
    observation_.push_back(state.gyro[0]);
    observation_.push_back(state.gyro[1]);
    observation_.push_back(state.gyro[2]);

    // 3. Velocity command (3) — rate-limited toward target
    vel_command_.step_filter(config_.policy_dt);
    observation_.push_back(vel_command_.vx);
    observation_.push_back(vel_command_.vy);
    observation_.push_back(vel_command_.vyaw);

    // 4. Gait phase (2): [cos φ_L, sin φ_L], gated to 0 when standing.
    // booster_gym's single-clock observation = cos/sin * (gait_freq > 0); the
    // gate is on (‖v_xy‖ > thr) OR (|ω_z| > thr).
    const float horiz_speed = std::hypot(vel_command_.vx, vel_command_.vy);
    gait_phase_.advance(config_.policy_dt,
                        std::max(horiz_speed, std::abs(vel_command_.vyaw)));
    const auto gait = gait_phase_.command();  // [cosL, cosR, sinL, sinR]
    const bool moving = (horiz_speed > gait_phase_.gate_speed_threshold) ||
                        (std::abs(vel_command_.vyaw) > gait_phase_.gate_speed_threshold);
    const float gate = moving ? 1.0f : 0.0f;
    observation_.push_back(gait[0] * gate);  // cos φ_L
    observation_.push_back(gait[2] * gate);  // sin φ_L

    // 5. Leg joint positions relative to default (12, policy/sim order)
    for (int a = 0; a < num_actions_; a++) {
        const int j = config_.action_to_joint_idx[a];
        observation_.push_back(state.joint_pos[j] - config_.robot.default_joint_pos[j]);
    }

    // 6. Leg joint velocities × 0.1 (12)
    for (int a = 0; a < num_actions_; a++) {
        const int j = config_.action_to_joint_idx[a];
        observation_.push_back(state.joint_vel[j] * 0.1f);
    }

    // 7. Last action (12, raw network output)
    for (int a = 0; a < num_actions_; a++) {
        observation_.push_back(last_action_[a]);
    }

#ifndef NDEBUG
    spec.validate_size(static_cast<int>(observation_.size()));
#endif
}
