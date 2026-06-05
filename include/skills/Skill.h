#pragma once
#include "RobotState.h"
#include "TaskConfig.h"
#include <array>

// A Skill controls a subset of the robot's joints.
//
// The composite Policy keeps one persistent target buffer (seeded with the
// default pose). Each policy step it calls compute() on every registered skill
// whose decimation divides the step counter, passing that shared buffer. A skill
// writes ONLY the joints it owns and leaves the rest untouched. Two consequences,
// matching colosseum_sdk's Simulation._tick:
//   - later skills win per joint (a HeadSkill registered after a LocomotionSkill
//     overwrites the head joints), and
//   - a skill skipped this step (decimation > 1) holds its previous targets,
//     because nothing overwrote them.
//
// Unlike colosseum_sdk, decimation here is measured in POLICY steps (get_action
// is the policy tick), not physics substeps — so a skill that runs every policy
// step uses decimation = 1.
class Skill {
public:
    virtual ~Skill() = default;

    int decimation = 1;  // policy steps between compute() calls (1 = every step)

    // Write position targets (rad, hardware joint order) for the joints this
    // skill controls into `targets`. Leave all other entries untouched.
    virtual void compute(const RobotState& state,
                         std::array<float, TaskConfig::NUM_JOINTS>& targets) = 0;

    virtual void reset() {}
};
