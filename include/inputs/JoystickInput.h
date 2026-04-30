#pragma once
#include "inputs/IInputSource.h"
#include <array>
#include <atomic>
#include <cmath>
#include <string>
#include <thread>

// IInputSource backed by a Linux joystick device (/dev/input/jsX).
// Runs a dedicated polling thread that updates normalized axis values
// atomically. Deadzone filtering (±10%) is applied on every update.
//
// Axis index mapping (Linux joystick event.number):
//   0 — left stick X    1 — left stick Y
//   2 — left trigger    3 — right stick X
//   4 — right stick Y   5 — right trigger
//   6 — d-pad X         7 — d-pad Y
class JoystickInput : public IInputSource {
public:
    explicit JoystickInput(std::string device);
    ~JoystickInput() override;

    float get_axis(int index)   const override;
    bool  get_button(int index) const override;
    void  stop()                      override;

    // Returns true if the device file can be opened (non-blocking check).
    static bool available(const std::string& device);

private:
    static constexpr int kMaxAxes    = 8;
    static constexpr int kMaxButtons = 16;
    static constexpr float kDeadzone = 0.1f;

    std::string        device_;
    std::atomic<bool>  stop_{false};
    std::thread        thread_;

    std::array<std::atomic<float>, kMaxAxes>   axes_{};
    std::array<std::atomic<bool>,  kMaxButtons> buttons_{};

    void pollLoop();
    static float normalize(int16_t raw) { return static_cast<float>(raw) / 32767.0f; }
    static float deadzone(float v)      { return std::fabs(v) < kDeadzone ? 0.0f : v; }
};
