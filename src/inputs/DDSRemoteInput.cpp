#include "inputs/DDSRemoteInput.h"

#include <booster/idl/b1/RemoteControllerState.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>
#include <thread>

// ── DDSRemoteInput ──────────────────────────────────────────────────────────

DDSRemoteInput::DDSRemoteInput() {
    poll_thread_ = std::thread(&DDSRemoteInput::pollLoop, this);
}

DDSRemoteInput::~DDSRemoteInput() {
    stop();
}

float DDSRemoteInput::get_axis(int index) const {
    if (index < 0 || index >= kMaxAxes) return 0.0f;
    std::lock_guard<std::mutex> lock(mutex_);
    return axes_[index];
}

bool DDSRemoteInput::get_button(int index) const {
    if (index < 0 || index >= kMaxButtons) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return buttons_[index];
}

void DDSRemoteInput::stop() {
    stop_.store(true);
    if (poll_thread_.joinable()) poll_thread_.join();
}

void DDSRemoteInput::pollLoop() {
    static constexpr const char* kTopic = "rt/remote_controller_state";

    try {
        booster::robot::ChannelFactory::Instance()->Init(0);
    } catch (...) {
        std::cerr << "[DDSRemoteInput] ChannelFactory::Init failed\n";
        return;
    }

    booster::robot::ChannelSubscriber<
        booster_interface::msg::RemoteControllerState> sub(kTopic);

    booster_interface::msg::RemoteControllerState latest_msg;
    std::mutex cb_mutex;
    bool has_msg = false;

    sub.InitChannel([&](const void* msg) {
        std::lock_guard<std::mutex> lock(cb_mutex);
        latest_msg = *static_cast<const booster_interface::msg::RemoteControllerState*>(msg);
        has_msg = true;
    });

    std::cout << "[DDSRemoteInput] Subscribed to " << kTopic << "\n" << std::flush;

    while (!stop_.load()) {
        {
            std::lock_guard<std::mutex> lock(cb_mutex);
            if (has_msg) {
                std::lock_guard<std::mutex> data_lock(mutex_);
                // Axes (floats, normalized -1..1 from DDS)
                axes_[0] = latest_msg.lx();  // left stick X
                axes_[1] = latest_msg.ly();  // left stick Y
                axes_[3] = latest_msg.rx();  // right stick X
                axes_[4] = latest_msg.ry();  // right stick Y
                // Triggers are bool in DDS — map to 0.0/1.0 axis
                axes_[2] = latest_msg.lt() ? 1.0f : 0.0f;  // left trigger
                axes_[5] = latest_msg.rt() ? 1.0f : 0.0f;  // right trigger

                // Buttons (match Linux evdev numbering used in main.cpp)
                buttons_[0] = latest_msg.a();    // kBtnA  = 0
                buttons_[1] = latest_msg.b();    // kBtnB  = 1
                buttons_[2] = latest_msg.x();    // X
                buttons_[3] = latest_msg.y();    // Y
                buttons_[4] = latest_msg.lb();   // LB
                buttons_[5] = latest_msg.rb();   // kBtnRB = 5
                buttons_[6] = latest_msg.back(); // Back
                buttons_[7] = latest_msg.start();// Start
                buttons_[8] = latest_msg.ls();   // Left stick
                buttons_[9] = latest_msg.rs();   // Right stick

                has_msg = false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "[DDSRemoteInput] Thread stopped.\n" << std::flush;
}
