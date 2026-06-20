#pragma once
#include "GainsLoader.h"
#include "RobotState.h"
#include "TaskConfig.h"
#include "skills/Skill.h"
#include <algorithm>
#include <array>
#include <memory>
#include <vector>

// Composite deployment policy: a list of Skills merged per joint each step.
//
// A Policy owns the robot/task config (gains, joint names, scene — read by main
// for publishCommand and by skills for obs/decode) and a list of Skills. Each
// step it runs the skills (respecting their decimation) over one persistent
// target buffer seeded with the default pose. Skills write only the joints they
// own; later skills win, skipped skills hold (see Skill.h).
//
// Subclasses are thin task factories: build the TaskConfig, then add_skill() the
// skills that make up the behavior (e.g. T1Velocity = LocomotionSkill + HeadSkill).
//
// Execution flow per step (main.cpp):
//   1. reset()           — reset all skills, reseed target buffer
//   2. get_action(state) — run due skills over the buffer, return 23 targets
class Policy {
public:
    explicit Policy(TaskConfig cfg) : config_(std::move(cfg)) {
        // Load per-joint deploy gains (PD, armature, friction, effort, default pose,
        // action scale) from the gains.yaml shipped next to the resolved ONNX — the
        // single source of truth shared with training. Subclasses only set joint
        // names and the action→joint mapping; the gains travel with the checkpoint.
        GainsMap gains =
            ModelConfig::parse_gains(gains_path_for_model(config_.model_path));
        apply_gains(gains, config_.robot);
        config_.action_scale = build_action_scale<TaskConfig::NUM_JOINTS>(
            gains, config_.robot.joint_names, config_.action_to_joint_idx);

        std::copy(config_.robot.default_joint_pos.begin(),
                  config_.robot.default_joint_pos.end(), merged_.begin());
    }

    virtual ~Policy() = default;

    void reset() {
        step_count_ = 0;
        std::copy(config_.robot.default_joint_pos.begin(),
                  config_.robot.default_joint_pos.end(), merged_.begin());
        for (auto& s : skills_) s->reset();
    }

    // Run one policy step. Returns joint position targets in hardware order.
    std::array<float, TaskConfig::NUM_JOINTS> get_action(const RobotState& state) {
        for (auto& s : skills_) {
            if (step_count_ % s->decimation == 0)
                s->compute(state, merged_);
        }
        step_count_++;
        return merged_;
    }

    const TaskConfig& config() const { return config_; }

protected:
    void add_skill(std::unique_ptr<Skill> s) { skills_.push_back(std::move(s)); }

    TaskConfig config_;
    std::vector<std::unique_ptr<Skill>> skills_;
    std::array<float, TaskConfig::NUM_JOINTS> merged_{};  // persistent merged targets
    int step_count_ = 0;
};
