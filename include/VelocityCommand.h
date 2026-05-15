#pragma once
#include <algorithm>
#include <cmath>

// Per-axis velocity limits. Embedded in TaskConfig and used to initialize
// VelocityCommand. Defaults match the hardcoded constants previously scattered
// across portal joystick loops (all 1.0 m/s or rad/s).
struct VelocityCommandConfig {
    float vx_max   = 1.0f;  // forward/backward (m/s)
    float vy_max   = 1.0f;  // lateral strafe    (m/s)
    float vyaw_max = 1.0f;  // yaw rotation      (rad/s)
    float ramp_vx   = 3.0f;  // max accel/decel on vx   (m/s²)
    float ramp_vy   = 3.0f;  // max accel/decel on vy   (m/s²)
    float ramp_vyaw = 3.0f;  // max accel/decel on vyaw (rad/s²)
};

// Current velocity command plus its limits. Embedded in RobotState.
//
// Two-layer design: set_normalized() writes an unfiltered target; step_filter()
// applies a rate-limiter that ramps the actual vx/vy/vyaw toward the target
// at a bounded rate, producing a smooth slope instead of an abrupt step.
struct VelocityCommand {
    float vx_max;    // forward/backward limit (m/s)
    float vy_max;    // lateral limit           (m/s)
    float vyaw_max;  // yaw limit               (rad/s)

    float ramp_vx;    // max |dvx/dt|   (m/s²)
    float ramp_vy;    // max |dvy/dt|   (m/s²)
    float ramp_vyaw;  // max |dvyaw/dt| (rad/s²)

    float target_vx   = 0.0f;  // raw desired forward command
    float target_vy   = 0.0f;  // raw desired lateral command
    float target_vyaw = 0.0f;  // raw desired yaw command

    float vx   = 0.0f;  // rate-limited forward command
    float vy   = 0.0f;  // rate-limited lateral command
    float vyaw = 0.0f;  // rate-limited yaw command

    explicit VelocityCommand(VelocityCommandConfig cfg = {})
        : vx_max(cfg.vx_max), vy_max(cfg.vy_max), vyaw_max(cfg.vyaw_max),
          ramp_vx(cfg.ramp_vx), ramp_vy(cfg.ramp_vy), ramp_vyaw(cfg.ramp_vyaw) {}

    // Called by joystick input: norm_* are deadzone-filtered values in [-1, 1].
    // Writes the raw target; step_filter() drives vx/vy/vyaw toward this target.
    void set_normalized(float norm_vx, float norm_vy, float norm_vyaw) {
        target_vx   = norm_vx   * vx_max;
        target_vy   = norm_vy   * vy_max;
        target_vyaw = norm_vyaw * vyaw_max;
    }

    // Apply rate-limiter: move vx/vy/vyaw toward target_* at bounded rate.
    void step_filter(float dt) {
        auto ramp_one = [](float& cur, float tgt, float max_rate, float dt_step) {
            float step = max_rate * dt_step;
            float err  = tgt - cur;
            if (err > step)
                cur += step;
            else if (err < -step)
                cur -= step;
            else
                cur = tgt;
        };
        ramp_one(vx,   target_vx,   ramp_vx,   dt);
        ramp_one(vy,   target_vy,   ramp_vy,   dt);
        ramp_one(vyaw, target_vyaw, ramp_vyaw, dt);
    }

    void stop() { target_vx = 0.0f; target_vy = 0.0f; target_vyaw = 0.0f; }
};
