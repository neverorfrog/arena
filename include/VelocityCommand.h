#pragma once
#include <algorithm>

// Per-axis velocity limits. Embedded in TaskConfig and used to initialize
// VelocityCommand. Defaults match the hardcoded constants previously scattered
// across portal joystick loops (all 1.0 m/s or rad/s).
struct VelocityCommandConfig {
    float vx_max   = 1.0f;  // forward/backward (m/s)
    float vy_max   = 1.0f;  // lateral strafe    (m/s)
    float vyaw_max = 1.0f;  // yaw rotation      (rad/s)
};

// Current velocity command plus its limits. Embedded in RobotState.
//
// Two write paths:
//   set_normalized() — joystick axis values in [-1, 1] are scaled by max.
//                      Replaces the per-portal `* VX_MAX` pattern.
//   Direct field writes (vx, vy, vyaw) — for keyboard input, which increments
//                      by a fixed step and clamps against the max fields.
struct VelocityCommand {
    float vx_max;    // forward/backward limit (m/s)
    float vy_max;    // lateral limit           (m/s)
    float vyaw_max;  // yaw limit               (rad/s)

    float vx   = 0.0f;  // current forward command
    float vy   = 0.0f;  // current lateral command
    float vyaw = 0.0f;  // current yaw command

    // Default constructor: limits from config (defaults to 1.0 m/s each).
    explicit VelocityCommand(VelocityCommandConfig cfg = {})
        : vx_max(cfg.vx_max), vy_max(cfg.vy_max), vyaw_max(cfg.vyaw_max) {}

    // Called by joystick input: norm_* are deadzone-filtered values in [-1, 1].
    void set_normalized(float norm_vx, float norm_vy, float norm_vyaw) {
        vx   = norm_vx   * vx_max;
        vy   = norm_vy   * vy_max;
        vyaw = norm_vyaw * vyaw_max;
    }

    void stop() { vx = 0.0f; vy = 0.0f; vyaw = 0.0f; }
};
