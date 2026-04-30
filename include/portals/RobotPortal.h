#pragma once
#include "portals/IPortal.h"
#include "RobotConfig.h"
#include "RobotState.h"
#include "booster/robot/b1/b1_loco_client.hpp"
#include "booster/robot/channel/channel_subscriber.hpp"
#include "booster/robot/channel/channel_publisher.hpp"
#include "booster/idl/b1/LowState.h"
#include "booster/idl/b1/LowCmd.h"
#include "booster/robot/b1/b1_api_const.hpp"

#include <array>
#include <atomic>
#include <booster/robot/b1/b1_loco_api.hpp>
#include <chrono>
#include <booster/robot/common/robot_shared.hpp>
#include <string>
#include <thread>

// IPortal implementation for the real Booster T1 robot via the Booster SDK.
//
// State is received asynchronously: the DDS subscriber fires lowStateCallback()
// whenever a new LowState message arrives (~50 Hz from the robot's onboard MCU).
// The callback reads IMU data, joint state, and derives projected_gravity from
// the quaternion. The control loop never blocks on state — updateState() is a
// no-op and getState() returns the most recent snapshot.
//
// Commands are sent as a LowCmd message with per-joint position targets and PD
// gains. The robot's onboard controller executes:
//   τ_i = kp_i * (target_i - pos_i) - kd_i * vel_i
//
// Lifecycle:
//   initialize() — DDS channel setup, B1LocoClient init, joystick thread start
//   publishCommand() — fill LowCmd and publish
//   tick() — sleep until next policy tick (50 Hz wall-clock)
//   shouldContinue() — always true; caller checks SIGINT flag
//
// Before the control loop the caller must switch the robot to kCustom mode via
// changeMode(RobotMode::kCustom) so the onboard safety layer accepts LowCmd.
class RobotPortal : public IPortal {

    public:
        explicit RobotPortal(float policy_dt);
        ~RobotPortal() override;

        void initialize() override;
        bool hasState() const override {
            return has_state_.load();
        }

        const RobotState& getState() const override {
            return state;
        }

        void updateState() override {}  // DDS callback updates state asynchronously.
        void publishCommand(const float* joint_targets, const float* kp, const float* kd) override;
        void tick() override;
        bool shouldContinue() const override { return true; }

        // Delete copy constructor and assignment operator
        RobotPortal(const RobotPortal&) = delete;
        RobotPortal& operator=(const RobotPortal&) = delete;

        const int changeMode(booster::robot::RobotMode mode) {
            return client.ChangeMode(mode);
        }

        // Move the robot to the safe prepare pose before starting the control
        // loop. Sends position commands at prepare_state stiffness/damping for
        // prep.duration_s seconds. Call after changeMode(kCustom).
        void prepare(const PrepareStateConfig<23>& prep);

    private:
        // State
        RobotState state;
        std::atomic<bool> has_state_{false};

        // LocoClient for synchronous RPC calls
        booster::robot::b1::B1LocoClient client;

        // DDS Channels
        ChannelSubscriber<booster_interface::msg::LowState> low_state_sub;
        void lowStateCallback(const booster_interface::msg::LowState& msg);
        ChannelPublisherPtr<booster_interface::msg::LowCmd> low_cmd_pub;

        // Utility functions
        std::array<float, 4> rpy_to_quat(const float rpy[3]) const;
        std::array<float, 3> compute_projected_gravity(const float q[4]) const;

        // Command
        booster_interface::msg::LowCmd cmd{};

        // Tick timing
        float policy_dt_;
        using Clock = std::chrono::steady_clock;
        Clock::time_point next_tick_{};
};
