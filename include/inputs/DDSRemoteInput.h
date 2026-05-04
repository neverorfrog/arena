#pragma once
#include "inputs/IInputSource.h"
#include <array>
#include <atomic>
#include <mutex>
#include <thread>

// IInputSource backed by the DDS rt/remote_controller_state topic.
// Used on the robot where the Spektrum transmitter is published by the
// MCU firmware over DDS, not as a Linux evdev device.
class DDSRemoteInput : public IInputSource {
public:
    DDSRemoteInput();
    ~DDSRemoteInput() override;

    float get_axis(int index)   const override;
    bool  get_button(int index) const override;
    void  stop()                      override;

private:
    static constexpr int kMaxAxes    = 8;
    static constexpr int kMaxButtons = 16;

    std::atomic<bool> stop_{false};
    std::thread       poll_thread_;
    mutable std::mutex mutex_;

    std::array<float, kMaxAxes>   axes_{};
    std::array<bool,  kMaxButtons> buttons_{};

    void pollLoop();
};
