#pragma once
#include "TaskConfig.h"
#include "skills/Skill.h"
#include <array>
#include <cmath>

// Scripted head look pose. Smoothly drives the two head joints toward a fixed,
// configurable target (yaw, pitch) — no network, no randomness. Port of the
// colosseum_sdk HeadSkill (exponential smoothing toward a preset).
//
// Replaces the old T1Velocity post_decode_action head-perturbation state machine
// (which replicated training-time HeadPerturbActionCfg). Deployment doesn't want
// random gaze; it wants a stable, settable look direction.
//
// Head joints are hardware indices 0 (AAHead_yaw) and 1 (Head_pitch), both with
// default_pos = 0. Register AFTER the locomotion skill so it owns these joints.
class HeadSkill : public Skill {
public:
    explicit HeadSkill(float yaw = 0.0f, float pitch = 0.0f)
        : yaw_target_(yaw), pitch_target_(pitch) {}

    // Set the look direction. Thread-safe for main-thread → loop use (plain floats).
    void set_pose(float yaw, float pitch) {
        yaw_target_   = yaw;
        pitch_target_ = pitch;
    }

    void reset() override {
        yaw_current_   = 0.0f;
        pitch_current_ = 0.0f;
    }

    void compute(const RobotState&, std::array<float, TaskConfig::NUM_JOINTS>& targets) override {
        const float alpha = 1.0f - std::exp(-POLICY_DT / TAU);
        yaw_current_   += alpha * (yaw_target_   - yaw_current_);
        pitch_current_ += alpha * (pitch_target_ - pitch_current_);
        targets[0] = yaw_current_;    // AAHead_yaw
        targets[1] = pitch_current_;  // Head_pitch
    }

private:
    static constexpr float POLICY_DT = 0.02f;  // policy step (s)
    static constexpr float TAU       = 0.4f;   // smoothing time constant (s)

    float yaw_target_, pitch_target_;
    float yaw_current_   = 0.0f;
    float pitch_current_ = 0.0f;
};
