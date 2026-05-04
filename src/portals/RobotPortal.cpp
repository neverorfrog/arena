#include "portals/RobotPortal.h"

#include <booster/robot/channel/channel_factory.hpp>
#include <cmath>
#include <iostream>

using booster::robot::ChannelPublisher;

RobotPortal::RobotPortal(float policy_dt)
    : state(),
      low_state_sub(booster::robot::b1::kTopicLowState),
      policy_dt_(policy_dt) {}

void RobotPortal::initialize() {
    booster::robot::ChannelFactory::Instance()->Init(0);
    client.Init();
    low_state_sub.InitChannel([this](const void* msg) {
        lowStateCallback(*static_cast<const booster_interface::msg::LowState*>(msg));
    });

    low_cmd_pub = std::make_shared<ChannelPublisher<booster_interface::msg::LowCmd>>(booster::robot::b1::kTopicJointCtrl);
    low_cmd_pub->InitChannel();
    std::cout << "[RobotPortal] Waiting 3s for DDS cmd subscriber discovery..." << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << " done\n" << std::flush;
    cmd.cmd_type(booster_interface::msg::CmdType::SERIAL);
    for (size_t i = 0; i < booster::robot::b1::kJointCnt; i++) {
      booster_interface::msg::MotorCmd motor_cmd;
      cmd.motor_cmd().push_back(motor_cmd);
    }

    next_tick_ = Clock::now();
}

void RobotPortal::publishCommand(const float* joint_targets, const float* kp, const float* kd) {
    for (int i = 0; i < booster::robot::b1::kJointCnt; i++) {
        cmd.motor_cmd().at(i).q(joint_targets[i]);
        cmd.motor_cmd().at(i).kp(kp[i]);
        cmd.motor_cmd().at(i).kd(kd[i]);
    }

    if (debug_counter_ % 50 == 0 && has_state_) {
        auto& s = state;
        std::cout << "[DIAG] step=" << debug_counter_
                  << " gyro=[" << s.gyro[0] << "," << s.gyro[1] << "," << s.gyro[2] << "]"
                  << " pg=[" << s.projected_gravity[0] << "," << s.projected_gravity[1] << "," << s.projected_gravity[2] << "]\n";
        const char* leg_names[] = {"LHP","LHR","LHY","LKN","LAP","LAR",
                                   "RHP","RHR","RHY","RKN","RAP","RAR"};
        for (int l = 0; l < 12; l++) {
            int j = 11 + l;
            std::cout << "  " << leg_names[l]
                      << " q=" << s.joint_pos[j]
                      << " v=" << s.joint_vel[j]
                      << " tgt=" << joint_targets[j]
                      << " kp=" << kp[j]
                      << " kd=" << kd[j] << "\n";
        }
    }
    debug_counter_++;

    low_cmd_pub->Write(&cmd);
}

void RobotPortal::smoothPrepare(const PrepareStateConfig<23>& prep) {
    if (!has_state_) return;
    float start_pos[23];
    for (int i = 0; i < 23; i++) start_pos[i] = state.joint_pos[i];

    auto start = Clock::now();
    auto duration = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<float>(prep.duration_s));

    float interp_targets[23];
    while (Clock::now() - start < duration) {
        float t = std::chrono::duration<float>(Clock::now() - start).count() / prep.duration_s;
        if (t > 1.0f) t = 1.0f;
        for (int i = 0; i < 23; i++)
            interp_targets[i] = start_pos[i] + t * (prep.joint_pos[i] - start_pos[i]);
        publishCommand(interp_targets, prep.stiffness.data(), prep.damping.data());
        std::this_thread::sleep_for(std::chrono::microseconds(2000));  // 500 Hz (matches booster_deploy)
    }
}

RobotPortal::~RobotPortal() = default;

void RobotPortal::tick() {
    next_tick_ += std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<float>(policy_dt_));
    std::this_thread::sleep_until(next_tick_);
}

void RobotPortal::lowStateCallback(const booster_interface::msg::LowState& msg) {
    if (!has_state_) std::cout << "[RobotPortal] First LowState received!\n" << std::flush;
    has_state_ = true;

    for (int i = 0; i < 3; i++) {
        state.rpy[i] = msg.imu_state().rpy()[i];
        state.gyro[i] = msg.imu_state().gyro()[i];
        state.acc[i] = msg.imu_state().acc()[i];
    }

    for (int i = 0; i < 23; i++) {
        state.joint_pos[i] = msg.motor_state_serial()[i].q();
        state.joint_vel[i] = msg.motor_state_serial()[i].dq();
        state.feedback_torque[i] = msg.motor_state_serial()[i].tau_est();
    }

    auto quat = rpy_to_quat(state.rpy);
    for (int i = 0; i < 4; i++) {
        state.root_quat[i] = quat[i];
    }
    auto pg = compute_projected_gravity(quat.data());
    for (int i = 0; i < 3; i++) {
        state.projected_gravity[i] = pg[i];
    }
}


std::array<float, 4> RobotPortal::rpy_to_quat(const float rpy[3]) const {
    float cr = cosf(rpy[0]*0.5f), sr = sinf(rpy[0]*0.5f);
    float cp = cosf(rpy[1]*0.5f), sp = sinf(rpy[1]*0.5f);
    float cy = cosf(rpy[2]*0.5f), sy = sinf(rpy[2]*0.5f);
    std::array<float, 4> q;
    q[0] = cr*cp*cy + sr*sp*sy;  // w
    q[1] = sr*cp*cy - cr*sp*sy;  // x
    q[2] = cr*sp*cy + sr*cp*sy;  // y
    q[3] = cr*cp*sy - sr*sp*cy;  // z
    return q;
}

std::array<float, 3> RobotPortal::compute_projected_gravity(const float q[4]) const {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    std::array<float, 3> pg;
    pg[0] =  2.0f * (w*y - x*z);
    pg[1] = -2.0f * (w*x + y*z);
    pg[2] = -(1.0f - 2.0f*(x*x + y*y));
    return pg;
}
