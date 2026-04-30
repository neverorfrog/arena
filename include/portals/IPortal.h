#pragma once
#include "RobotState.h"

// Backend-agnostic portal interface.
// Implementations: BoosterPortal (real robot via DDS), MujocoPortal (standalone sim).
class IPortal {
public:
    virtual ~IPortal() = default;

    // One-time setup (SDK init, model load, thread start).
    virtual void initialize() = 0;

    // True once at least one state message has been received.
    virtual bool hasState() const = 0;

    // Current robot state (joint pos/vel, IMU, projected gravity, velocity cmd).
    virtual const RobotState& getState() const = 0;

    // Re-read state from the underlying source.
    // Booster: noop (state updated asynchronously via DDS callback).
    // MuJoCo: copies from mj_data after the last physics step.
    virtual void updateState() = 0;

    // Send joint position targets + per-joint PD gains.
    virtual void publishCommand(const float* targets,
                                const float* kp,
                                const float* kd) = 0;

    // Called once per policy cycle after publishCommand().
    // Booster: sleeps until next 50 Hz tick.
    // MuJoCo: steps physics (decimation times) + syncs viewer + sleeps.
    virtual void tick() = 0;

    // Returns false when the session should end.
    // Booster: always true (caller checks SIGINT flag).
    // MuJoCo: false when the viewer window is closed.
    virtual bool shouldContinue() const = 0;
};
