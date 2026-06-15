#pragma once
#include "skills/LocomotionSkill.h"

// Velocity-tracking locomotion skill for the Booster T1 12-DOF (legs-only)
// policy. Reuses LocomotionSkill's engine / input / inference / action-decode
// machinery (action count = engine output_dim = 12) and only overrides the observation
// layout, which differs from the 23-DOF body policy in term order, dimensions
// and scaling.
//
// Observation layout (47 elements, must match colosseum observation_cfg.py):
//   [0:3]    projected gravity in body frame
//   [3:6]    base angular velocity (gyro, rad/s)
//   [6:9]    velocity command [vx, vy, omega]
//   [9:11]   gait phase [cos φ_L, sin φ_L], gated to 0 when standing
//   [11:23]  leg joint positions relative to default (12, sim order)
//   [23:35]  leg joint velocities × 0.1 (12, sim order)
//   [35:47]  last action (12, raw network output)
//
// Action decoding (per leg joint a, hardware index j = action_to_joint_idx[a]):
//   targets[j] = net_out[a] * action_scale[a] + default_joint_pos[j]
// The 11 upper-body joints no skill writes stay pinned at their defaults.
class LocomotionSkill12 : public LocomotionSkill {
public:
    explicit LocomotionSkill12(const TaskConfig& cfg);

protected:
    void build_observation(const RobotState& state) override;
};
