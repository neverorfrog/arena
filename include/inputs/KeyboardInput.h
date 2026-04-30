#pragma once
#include "inputs/IInputSource.h"
#include <array>
#include <atomic>
#include <thread>
#include <termios.h>

// IInputSource backed by terminal keyboard input.
// Used as a fallback when no joystick is available.
//
// Key → axis mapping (synthesizes joystick-compatible axis indices):
//   W / S  → axis 1 (left stick Y):  +step / -step
//   A / D  → axis 0 (left stick X):  +step / -step
//   Q / E  → axis 3 (right stick X): +step / -step
//   Space  → all axes → 0
//
// Values are accumulated (W increments, S decrements) and clamped to [-1, 1],
// so they behave like a stepped joystick rather than a momentary button.
class KeyboardInput : public IInputSource {
public:
    KeyboardInput();
    ~KeyboardInput() override;

    float get_axis(int index)   const override;
    bool  get_button(int index) const override { return false; }
    void  stop()                      override;

private:
    static constexpr int   kMaxAxes = 8;
    static constexpr float kStep    = 0.1f;

    std::atomic<bool>  stop_{false};
    std::thread        thread_;
    termios            old_tio_{};

    std::array<std::atomic<float>, kMaxAxes> axes_{};

    void listenLoop();
    void applyKey(char key);
};
