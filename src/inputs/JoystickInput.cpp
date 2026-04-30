#include "inputs/JoystickInput.h"
#include "inputs/IInputSource.h"
#include "inputs/KeyboardInput.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <poll.h>
#include <unistd.h>

// ──────────────────────────────────────────────────────────────────────────────
// JoystickInput
// ──────────────────────────────────────────────────────────────────────────────

JoystickInput::JoystickInput(std::string device) : device_(std::move(device)) {
    for (auto& a : axes_)   a.store(0.0f);
    for (auto& b : buttons_) b.store(false);
    thread_ = std::thread(&JoystickInput::pollLoop, this);
}

JoystickInput::~JoystickInput() {
    stop();
}

float JoystickInput::get_axis(int index) const {
    if (index < 0 || index >= kMaxAxes) return 0.0f;
    return axes_[index].load();
}

bool JoystickInput::get_button(int index) const {
    if (index < 0 || index >= kMaxButtons) return false;
    return buttons_[index].load();
}

void JoystickInput::stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
}

bool JoystickInput::available(const std::string& device) {
    int fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) return false;
    close(fd);
    return true;
}

void JoystickInput::pollLoop() {
    int fd = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[JoystickInput] Cannot open " << device_
                  << ": " << strerror(errno) << "\n";
        return;
    }
    std::cout << "[JoystickInput] Opened " << device_ << "\n";

    pollfd pfd{fd, POLLIN, 0};
    js_event event;

    while (!stop_.load()) {
        if (poll(&pfd, 1, 50) <= 0) continue;
        if (read(fd, &event, sizeof(event)) != sizeof(event)) continue;

        const uint8_t type = event.type & ~JS_EVENT_INIT;
        if (type == JS_EVENT_AXIS && event.number < kMaxAxes) {
            axes_[event.number].store(deadzone(normalize(event.value)));
        } else if (type == JS_EVENT_BUTTON && event.number < kMaxButtons) {
            buttons_[event.number].store(event.value != 0);
        }
    }

    close(fd);
    std::cout << "[JoystickInput] Thread stopped.\n";
}

// ──────────────────────────────────────────────────────────────────────────────
// Factory
// ──────────────────────────────────────────────────────────────────────────────

std::unique_ptr<IInputSource> create_input_source() {
    const char* env = std::getenv("JOYSTICK_DEVICE");
    std::string device = env ? env : "/dev/input/js2";

    if (JoystickInput::available(device)) {
        return std::make_unique<JoystickInput>(std::move(device));
    }

    std::cout << "[input] Joystick not available at " << device
              << " — falling back to keyboard.\n";
    return std::make_unique<KeyboardInput>();
}
