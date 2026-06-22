#pragma once
#include "portals/IPortal.h"
#include "TaskConfig.h"
#include <mujoco/mujoco.h>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <utility>
#include <string>
#include <thread>

// Forward-declare GLFWwindow so this header doesn't drag in all of GLFW.
// The full definition is only needed in MujocoPortal.cpp.
struct GLFWwindow;

// Simulation tuning parameters for the MuJoCo backend.
// physics_dt should satisfy: decimation * physics_dt == policy_dt (0.02 s).
// Matches training: timestep=0.005s, decimation=4 (200 Hz physics).
struct MujocoConfig {
    float init_height = 0.66f;   // base spawn height (m) — matches colosseum
    int   decimation  = 4;        // physics steps per policy step
    float physics_dt  = 0.005f;  // seconds per physics step (200 Hz)

    // Sim-to-sim latency injection (policy steps). MujocoPortal otherwise has
    // ZERO latency, unlike the real robot's FastDDS transport (~2-3 steps) and
    // the training obs-delay DR — which is why the on-hardware ankle ring never
    // reproduced here. obs_delay_steps delays what getState() exposes to the
    // policy; action_delay_steps delays when published commands reach physics.
    // Overridable at runtime via ARENA_OBS_DELAY / ARENA_ACT_DELAY.
    int   obs_delay_steps    = 0;
    int   action_delay_steps = 1;

    // Settle at the prepare pose with prepare gains before handing to the policy,
    // reproducing the hardware startup handoff (prepare_pose -> policy commands
    // default_joint_pos under a gain switch). Disable via ARENA_NO_PREPARE.
    bool  prepare = true;
};

// IPortal implementation for sim-to-sim testing using MuJoCo + GLFW.
//
// Loads two MJCF files at runtime (scene + robot), merges them, and compiles a
// single mjModel. Actuators are NOT taken from the XML — the portal creates one
// position actuator per joint programmatically, with Kp/Kd from TaskConfig.
// This guarantees the PD gains exactly match training, regardless of what the
// XML contains.
//
// State update flow (per policy step):
//   1. tick() runs `decimation` mj_step() calls at physics_dt  (200 Hz)
//   2. updateState() reads qpos/qvel and IMU sensor data into RobotState
//      — joint_pos/vel: indexed via joint_qpos_idx_ / joint_dof_idx_
//      — root_quat: from the "orientation" sensor (gyroscope frame)
//      — projected_gravity: computed from quaternion
//
// Input: joystick (evdev) or keyboard fallback, running in a dedicated thread.
// Viewer: GLFW window with mjv/mjr rendering, synced once per policy tick.
//
// shouldContinue() returns false when the GLFW window is closed.
class MujocoPortal : public IPortal {
public:
    explicit MujocoPortal(MujocoConfig cfg, const TaskConfig& task_cfg);
    ~MujocoPortal() override;

    void initialize()                                           override;
    bool hasState()                    const                   override { return has_state_.load(); }
    const RobotState& getState()       const                   override { return state_; }
    void updateState()                                         override;
    void publishCommand(const float* targets,
                        const float* kp,
                        const float* kd)                       override;
    void tick()                                                override;
    bool shouldContinue()              const                   override;

private:
    MujocoConfig     cfg_;
    const TaskConfig& task_cfg_;

    mjModel* mj_model_ = nullptr;
    mjData*  mj_data_  = nullptr;

    // MuJoCo viewer
    mjvCamera   cam_{};
    mjvOption   opt_{};
    mjvScene    scn_{};
    mjrContext  con_{};
    GLFWwindow* window_ = nullptr;

    RobotState        state_;
    std::atomic<bool> has_state_{false};

    // Per-joint indices in qpos / dof arrays (resolved at init from joint names)
    std::array<int, TaskConfig::NUM_JOINTS> joint_qpos_idx_{};
    std::array<int, TaskConfig::NUM_JOINTS> joint_dof_idx_{};
    // Sensor offsets in sensordata[]
    int gyro_offset_ = -1;
    int quat_offset_ = -1;

    // Debug: when set (env ARENA_HOLD_POSE), ignore policy commands and hold
    // default_joint_pos with the trained gains. Isolates plant instability
    // (e.g. ankle-roll buzz) from the obs→policy→action loop.
    bool hold_pose_ = false;

    // Pending joint commands (written by publishCommand, consumed by tick).
    std::array<double, TaskConfig::NUM_JOINTS> target_{};
    std::array<double, TaskConfig::NUM_JOINTS> kp_{};
    std::array<double, TaskConfig::NUM_JOINTS> kd_{};

    // Latency injection (see MujocoConfig). Resolved from cfg_ + env at init.
    int  obs_delay_steps_    = 0;
    int  action_delay_steps_ = 0;
    bool prepare_            = true;
    // Ring of recent raw states; getState() exposes the one obs_delay_steps_ back.
    std::deque<RobotState> obs_ring_;
    // Ring of recent commands; tick() applies the one action_delay_steps_ back.
    struct DelayedCommand {
        std::array<double, TaskConfig::NUM_JOINTS> target, kp, kd;
    };
    std::deque<DelayedCommand> cmd_ring_;

    // Tick timing
    using Clock = std::chrono::steady_clock;
    Clock::time_point next_tick_;


    // Internal helpers
    void setupViewer();
    void stepPhysics();   // one policy step: decimation PD substeps (no render/sleep)
    void runPrepare();    // settle at prepare pose with prepare gains before policy
    static std::array<float, 3> projectedGravity(const double q[4]);
};
