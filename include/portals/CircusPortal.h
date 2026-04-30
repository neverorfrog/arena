#pragma once
#include "portals/IPortal.h"
#include "TaskConfig.h"
#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

struct CircusConfig {
    std::string host      = "127.0.0.1";
    int         port      = 5555;
    std::string robot_name = "T1";  // must match the name configured in Circus
};

// IPortal implementation that connects to a running Circus simulation.
//
// Protocol (from RobotManager.cpp):
//   connect → send robot_name (msgpack string) → recv initial state
//   loop:  send {"robot_name": str, "joint_torques": [23]}  (raw msgpack)
//          recv  [uint32 big-endian size][msgpack state map]
//
// Since Circus uses a torque interface, this portal computes PD torques:
//   τ_i = kp_i*(target_i - pos_i) - kd_i*vel_i   clamped to ±effort_limit_i
class CircusPortal : public IPortal {
public:
    explicit CircusPortal(CircusConfig cfg, const TaskConfig& task_cfg);
    ~CircusPortal() override;

    void initialize()                                             override;
    bool hasState()                      const                   override { return has_state_.load(); }
    const RobotState& getState()         const                   override { return state_; }
    void updateState()                                           override {}  // state is updated in tick()
    void publishCommand(const float* targets,
                        const float* kp,
                        const float* kd)                         override;
    void tick()                                                  override;
    bool shouldContinue()                const                   override { return fd_ >= 0; }

private:
    CircusConfig      cfg_;
    const TaskConfig& task_cfg_;
    int               fd_ = -1;

    RobotState        state_;
    std::atomic<bool> has_state_{false};

    // Pending position targets + gains (set by publishCommand, used in tick)
    std::array<float, TaskConfig::NUM_JOINTS> targets_{};
    std::array<float, TaskConfig::NUM_JOINTS> kp_{};
    std::array<float, TaskConfig::NUM_JOINTS> kd_{};

    // Tick timing
    using Clock = std::chrono::steady_clock;
    Clock::time_point next_tick_;

    // Networking helpers
    bool sendMsgpack(const void* data, size_t len);
    bool recvSized(std::vector<char>& buf);  // reads [uint32 size][data]

    // Parse incoming Circus state message into state_
    void parseState(const std::vector<char>& buf);

    static std::array<float, 3> projectedGravity(const float q[4]);
};
