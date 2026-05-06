#include "inputs/KeyboardInput.h"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sys/select.h>
#include <unistd.h>

KeyboardInput::KeyboardInput() {
    for (auto& a : axes_) a.store(0.0f);

    if (!isatty(STDIN_FILENO)) {
        std::cerr << "[KeyboardInput] stdin is not a TTY; keyboard input disabled.\n";
        return;
    }

    if (tcgetattr(STDIN_FILENO, &old_tio_) != 0) {
        std::cerr << "[KeyboardInput] tcgetattr failed; keyboard input disabled.\n";
        return;
    }

    termios raw = old_tio_;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 1;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        std::cerr << "[KeyboardInput] tcsetattr failed; keyboard input disabled.\n";
        return;
    }

    std::cout << "[KeyboardInput] w/s forward-back | a/d left-right | q/e yaw | space stop\n"
              << "[KeyboardInput] combos: 3(RB) r(RT) 1(A)=activate | 3 r 2(B)=deactivate | 3 r=estop\n";
    thread_ = std::thread(&KeyboardInput::listenLoop, this);
}

KeyboardInput::~KeyboardInput() {
    stop();
}

float KeyboardInput::get_axis(int index) const {
    if (index < 0 || index >= kMaxAxes) return 0.0f;
    return axes_[index].load();
}

bool KeyboardInput::get_button(int index) const {
    if (index < 0 || index >= kMaxButtons) return false;
    return buttons_[index].load();
}

void KeyboardInput::stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
}

void KeyboardInput::applyKey(char key) {
    // Axis 1 (left stick Y): W = forward (+), S = backward (-)
    // Axis 0 (left stick X): A = left (+), D = right (-)
    // Axis 3 (right stick X): Q = left (+), E = right (-)
    auto& ax1 = axes_[1]; auto& ax0 = axes_[0]; auto& ax3 = axes_[3];
    auto& ax5 = axes_[5];
    switch (key) {
        case 'w': ax1.store(std::min(ax1.load() + kStep,  1.0f)); break;
        case 's': ax1.store(std::max(ax1.load() - kStep, -1.0f)); break;
        case 'a': ax0.store(std::min(ax0.load() + kStep,  1.0f)); break;
        case 'd': ax0.store(std::max(ax0.load() - kStep, -1.0f)); break;
        case 'q': ax3.store(std::min(ax3.load() + kStep,  1.0f)); break;
        case 'e': ax3.store(std::max(ax3.load() - kStep, -1.0f)); break;
        case ' ':
            for (auto& a : axes_) a.store(0.0f);
            break;
        // Button toggles (joystick combo simulation)
        case '1': buttons_[0].store(!buttons_[0].load()); break;  // A (play)
        case '2': buttons_[1].store(!buttons_[1].load()); break;  // B (manual)
        case '3': buttons_[5].store(!buttons_[5].load()); break;  // RB
        case 'r': ax5.store(ax5.load() > 0.5f ? 0.0f : 1.0f); break; // RT toggle
        default: break;
    }
}

void KeyboardInput::listenLoop() {
    while (!stop_.load()) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        timeval timeout{};
        timeout.tv_sec  = 0;
        timeout.tv_usec = 100000;

        if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) <= 0) continue;
        if (!FD_ISSET(STDIN_FILENO, &readfds)) continue;

        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) != 1) continue;
        if (ch == '\n' || ch == '\r' || ch == '\x03') continue;
        applyKey(ch);
    }
}
