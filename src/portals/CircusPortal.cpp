#include "portals/CircusPortal.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <msgpack.hpp>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

// ──────────────────────────────────────────────────────────────────────────────
// Construction / destruction
// ──────────────────────────────────────────────────────────────────────────────

CircusPortal::CircusPortal(CircusConfig cfg, const TaskConfig& task_cfg)
    : cfg_(std::move(cfg)), task_cfg_(task_cfg) {}

CircusPortal::~CircusPortal() {
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
}

// ──────────────────────────────────────────────────────────────────────────────
// initialize()
// ──────────────────────────────────────────────────────────────────────────────

void CircusPortal::initialize() {
    // 1. Open TCP socket.
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0)
        throw std::runtime_error("CircusPortal: socket() failed: " + std::string(strerror(errno)));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(cfg_.port);
    if (inet_pton(AF_INET, cfg_.host.c_str(), &addr.sin_addr) <= 0)
        throw std::runtime_error("CircusPortal: invalid host: " + cfg_.host);

    if (connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("CircusPortal: connect to " + cfg_.host + ":" +
                                 std::to_string(cfg_.port) + " failed: " + strerror(errno));

    std::cout << "[CircusPortal] Connected to Circus at "
              << cfg_.host << ":" << cfg_.port << "\n";

    // 2. Handshake: send robot name as a msgpack string.
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, cfg_.robot_name);
    if (!sendMsgpack(sbuf.data(), sbuf.size()))
        throw std::runtime_error("CircusPortal: failed to send handshake");

    // 3. Receive initial state (Circus sends [size][msgpack] format).
    std::vector<char> buf;
    if (!recvSized(buf))
        throw std::runtime_error("CircusPortal: failed to receive initial state");
    parseState(buf);
    has_state_ = true;

    next_tick_ = Clock::now();
    std::cout << "[CircusPortal] Ready. Robot: " << cfg_.robot_name << "\n";
}

// ──────────────────────────────────────────────────────────────────────────────
// publishCommand()  — store position targets + gains for next tick
// ──────────────────────────────────────────────────────────────────────────────

void CircusPortal::publishCommand(const float* targets, const float* kp, const float* kd) {
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        targets_[i] = targets[i];
        kp_[i]      = kp[i];
        kd_[i]      = kd[i];
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// tick()  — compute PD torques, send to Circus, receive updated state
// ──────────────────────────────────────────────────────────────────────────────

void CircusPortal::tick() {
    // 1. Compute PD torques from position targets.
    std::vector<double> torques(TaskConfig::NUM_JOINTS);
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        double tau = kp_[i] * (targets_[i] - state_.joint_pos[i])
                   - kd_[i] * state_.joint_vel[i];
        double limit = task_cfg_.robot.effort_limit[i];
        torques[i] = std::clamp(tau, -limit, limit);
    }

    // 2. Pack and send {"robot_name": str, "joint_torques": [23]}.
    msgpack::zone z;
    std::map<std::string, msgpack::object> msg;
    msg["robot_name"]   = msgpack::object(cfg_.robot_name, z);
    msg["joint_torques"] = msgpack::object(torques, z);

    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, msg);
    if (!sendMsgpack(sbuf.data(), sbuf.size())) {
        std::cerr << "[CircusPortal] Send failed — closing.\n";
        close(fd_); fd_ = -1;
        return;
    }

    // 3. Receive updated state.
    std::vector<char> buf;
    if (!recvSized(buf)) {
        std::cerr << "[CircusPortal] Recv failed — closing.\n";
        close(fd_); fd_ = -1;
        return;
    }
    parseState(buf);

    // 4. Sleep to maintain policy rate.
    next_tick_ += std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<float>(task_cfg_.policy_dt));
    std::this_thread::sleep_until(next_tick_);
}

// ──────────────────────────────────────────────────────────────────────────────
// sendMsgpack()  — send raw msgpack bytes (no size prefix)
// ──────────────────────────────────────────────────────────────────────────────

bool CircusPortal::sendMsgpack(const void* data, size_t len) {
    size_t total = 0;
    const char* ptr = static_cast<const char*>(data);
    while (total < len) {
        ssize_t n = send(fd_, ptr + total, len - total, 0);
        if (n <= 0) return false;
        total += n;
    }
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// recvSized()  — receive [uint32 big-endian size][msgpack data]
// ──────────────────────────────────────────────────────────────────────────────

bool CircusPortal::recvSized(std::vector<char>& buf) {
    // Read 4-byte size header.
    uint32_t net_size = 0;
    size_t   header_read = 0;
    while (header_read < sizeof(net_size)) {
        ssize_t n = recv(fd_, reinterpret_cast<char*>(&net_size) + header_read,
                         sizeof(net_size) - header_read, 0);
        if (n <= 0) return false;
        header_read += n;
    }
    uint32_t size = ntohl(net_size);
    if (size == 0 || size > 4 * 1024 * 1024) return false;  // sanity check

    // Read payload.
    buf.resize(size);
    size_t total = 0;
    while (total < size) {
        ssize_t n = recv(fd_, buf.data() + total, size - total, 0);
        if (n <= 0) return false;
        total += n;
    }
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// parseState()  — decode Circus state message into RobotState
// ──────────────────────────────────────────────────────────────────────────────

void CircusPortal::parseState(const std::vector<char>& buf) {
    msgpack::object_handle oh = msgpack::unpack(buf.data(), buf.size());
    auto top = oh.get().as<std::map<std::string, msgpack::object>>();

    // joints: {position, velocity, ...}
    auto it_joints = top.find("joints");
    if (it_joints != top.end()) {
        auto joints_map = it_joints->second.as<std::map<std::string, msgpack::object>>();

        auto pos = joints_map.at("position").as<std::vector<double>>();
        auto vel = joints_map.at("velocity").as<std::vector<double>>();
        for (int i = 0; i < TaskConfig::NUM_JOINTS && i < (int)pos.size(); i++) {
            state_.joint_pos[i] = static_cast<float>(pos[i]);
            state_.joint_vel[i] = static_cast<float>(vel[i]);
        }
    }

    // imu: {angular_velocity, linear_acceleration}
    auto it_imu = top.find("imu");
    if (it_imu != top.end()) {
        auto imu_map = it_imu->second.as<std::map<std::string, msgpack::object>>();

        auto gyro = imu_map.at("angular_velocity").as<std::vector<double>>();
        for (int k = 0; k < 3 && k < (int)gyro.size(); k++)
            state_.gyro[k] = static_cast<float>(gyro[k]);

        auto acc = imu_map.at("linear_acceleration").as<std::vector<double>>();
        for (int k = 0; k < 3 && k < (int)acc.size(); k++)
            state_.acc[k] = static_cast<float>(acc[k]);
    }

    // pose: {position: [3], orientation: [4] (w,x,y,z)}
    auto it_pose = top.find("pose");
    if (it_pose != top.end()) {
        auto pose_map = it_pose->second.as<std::map<std::string, msgpack::object>>();

        auto ori = pose_map.at("orientation").as<std::vector<double>>();
        if (ori.size() >= 4) {
            float q[4];
            for (int k = 0; k < 4; k++) {
                state_.root_quat[k] = q[k] = static_cast<float>(ori[k]);
            }
            auto pg = projectedGravity(q);
            for (int k = 0; k < 3; k++) state_.projected_gravity[k] = pg[k];
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// joystickLoop()
// ──────────────────────────────────────────────────────────────────────────────

// ──────────────────────────────────────────────────────────────────────────────
// projectedGravity()
// ──────────────────────────────────────────────────────────────────────────────

std::array<float, 3> CircusPortal::projectedGravity(const float q[4]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    return {
         2.0f * (w*y - x*z),
        -2.0f * (w*x + y*z),
        -(1.0f - 2.0f*(x*x + y*y))
    };
}
