#pragma once
#include "portals/IPortal.h"
#include "TaskConfig.h"
#include <mujoco/mujoco.h>
#include <array>
#include <atomic>
#include <chrono>
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
    float init_height = 0.65f;   // base spawn height (m)
    int   decimation  = 4;        // physics steps per policy step
    float physics_dt  = 0.005f;  // seconds per physics step (200 Hz)
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

    // Pending joint commands (written by publishCommand, consumed by tick).
    std::array<double, TaskConfig::NUM_JOINTS> target_{};
    std::array<double, TaskConfig::NUM_JOINTS> kp_{};
    std::array<double, TaskConfig::NUM_JOINTS> kd_{};

    // Tick timing
    using Clock = std::chrono::steady_clock;
    Clock::time_point next_tick_;


    // Internal helpers
    void setupViewer();
    static std::array<float, 3> projectedGravity(const double q[4]);
};
