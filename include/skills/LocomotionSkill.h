#pragma once
#include "GaitPhaseCommand.h"
#include "RobotData.h"
#include "RobotState.h"
#include "TaskConfig.h"
#include "VelocityCommand.h"
#include "engines/IInferenceEngine.h"
#include "inputs/IInputSource.h"
#include "skills/Skill.h"
#include <array>
#include <memory>
#include <vector>

// Velocity-tracking locomotion skill for the Booster T1 23-DOF humanoid.
//
// Owns the ONNX/TRT inference engine, the joystick input source, the velocity
// command + gait-phase clocks, and the heading lock. Controls the 21 body joints
// (hardware indices 2..22); the 2 head joints are left to a HeadSkill.
//
// Observation layout (80 elements, must match training VelocityPolicy):
//   [0:3]   base angular velocity (gyro, rad/s)
//   [3:6]   projected gravity in body frame
//   [6:29]  joint positions relative to default pose (rad)
//   [29:52] joint velocities (rad/s)
//   [52:73] last action (raw network output, 21-DOF)
//   [73:76] velocity command [vx, vy, omega]
//   [76:80] gait phase [cos(φ_L), cos(φ_R), sin(φ_L), sin(φ_R)]
//
// Action decoding (per body joint a, hardware index j = a + 2):
//   targets[j] = net_out[a] * action_scale[a] + default_joint_pos[j]
class LocomotionSkill : public Skill {
public:
    explicit LocomotionSkill(const TaskConfig& cfg);
    ~LocomotionSkill() override;

    void compute(const RobotState& state,
                 std::array<float, TaskConfig::NUM_JOINTS>& targets) override;
    void reset() override;

protected:
    void  update_input();
    // Build the observation vector for this policy. The base implementation is
    // the 23-DOF body layout; LocomotionSkill12 overrides it for the legs-only
    // 12-DOF layout. Writes into observation_.
    virtual void build_observation(const RobotState& state);
    static float wrap_to_pi(float a);

    TaskConfig config_;
    RobotData<TaskConfig::NUM_JOINTS> robot_data_;
    std::unique_ptr<IInferenceEngine> engine_;
    std::unique_ptr<IInputSource>     input_source_;

    int num_actions_ = 0;  // policy output dimension, from engine_->output_dim()
    std::vector<float> observation_;
    std::vector<float> last_action_;  // sized num_actions_, raw network output

    // Task-specific command. Limits (1 m/s, 1 m/s, 1 rad/s) match training.
    VelocityCommand  vel_command_{VelocityCommandConfig{1.0f, 1.0f, 1.0f}};
    GaitPhaseCommand gait_phase_{};

    // Heading lock — mirrors training rel_heading_envs=1.0 play mode.
    float last_yaw_       = 0.0f;
    float heading_target_ = 0.0f;
    bool  heading_locked_ = false;
};
