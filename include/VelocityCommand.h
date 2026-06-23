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
    float ramp_vx   = 5.0f;  // max accel/decel on vx   (m/s²)
    float ramp_vy   = 5.0f;  // max accel/decel on vy   (m/s²)
    float ramp_vyaw = 5.0f;  // max accel/decel on vyaw (rad/s²)
    float soft_stop_speed = 0.055f;  // low speed held before final stop (0 = disabled)
    float soft_stop_hold  = 0.75f;    // time to decelerate from soft_stop_speed to 0 (s)
};

// Current velocity command plus its limits. Embedded in RobotState.
//
// Two-layer design: update_input() writes target_vx/target_vy/target_vyaw;
// step_filter() applies a rate-limiter that ramps the actual vx/vy/vyaw toward
// the target at a bounded rate, producing a smooth slope instead of an abrupt
// step.
//
// Soft stop: when decelerating to zero, step_filter uses two-phase ramping to
// prevent the robot from being commanded to stop abruptly. Phase 1 decelerates
// at the normal ramp rate down to soft_stop_speed (e.g. 0.055 m/s). Phase 2
// uses a much slower final-approach rate so the robot takes ~2 steps before
// reaching zero. Set soft_stop_speed=0 to disable.
struct VelocityCommand {
    float vx_max;    // forward/backward limit (m/s)
    float vy_max;    // lateral limit           (m/s)
    float vyaw_max;  // yaw limit               (rad/s)

    float ramp_vx;    // max |dvx/dt|   (m/s²)
    float ramp_vy;    // max |dvy/dt|   (m/s²)
    float ramp_vyaw;  // max |dvyaw/dt| (rad/s²)

    float soft_stop_speed;  // low speed held before final stop (m/s or rad/s)
    float soft_stop_hold;   // time to decelerate from soft_stop_speed to zero

    float target_vx   = 0.0f;  // raw desired forward command
    float target_vy   = 0.0f;  // raw desired lateral command
    float target_vyaw = 0.0f;  // raw desired yaw command

    float vx   = 0.0f;  // rate-limited forward command
    float vy   = 0.0f;  // rate-limited lateral command
    float vyaw = 0.0f;  // rate-limited yaw command

    explicit VelocityCommand(VelocityCommandConfig cfg = {})
        : vx_max(cfg.vx_max), vy_max(cfg.vy_max), vyaw_max(cfg.vyaw_max),
          ramp_vx(cfg.ramp_vx), ramp_vy(cfg.ramp_vy), ramp_vyaw(cfg.ramp_vyaw),
          soft_stop_speed(cfg.soft_stop_speed), soft_stop_hold(cfg.soft_stop_hold) {}

    // Called by joystick input: norm_* are deadzone-filtered values in [-1, 1].
    // Writes the raw target; step_filter() drives vx/vy/vyaw toward this target.
    void set_normalized(float norm_vx, float norm_vy, float norm_vyaw) {
        target_vx   = norm_vx   * vx_max;
        target_vy   = norm_vy   * vy_max;
        target_vyaw = norm_vyaw * vyaw_max;
    }

    // Apply rate-limiter: move vx/vy/vyaw toward target_* at bounded rate.
    // When the target is zero, uses a slower final-approach ramp to avoid
    // commanding an abrupt stop that could cause vibration or tilt.
    void step_filter(float dt) {
        auto ramp_one = [this](float& cur, float tgt, float max_rate, float dt_step) {
            float rate = max_rate;
            float soft_rate = (soft_stop_speed > 0.0f) ? soft_stop_speed / soft_stop_hold : 0.0f;

            if (tgt == 0.0f && soft_rate > 0.0f && std::fabs(cur) < soft_stop_speed && cur != 0.0f) {
                rate = soft_rate;
            }

            float step = rate * dt_step;
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
