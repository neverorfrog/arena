#pragma once
#include <memory>

// Generic hardware input abstraction. Provides raw axis and button values
// without any interpretation. The policy maps these to task-specific
// quantities (e.g. velocity commands, mode switches).
//
// Axis values are normalized to [-1, 1] with deadzone filtering applied.
// Indices follow the Linux joystick event numbering (axis 0 = left stick X,
// axis 1 = left stick Y, axis 3 = right stick X, etc.).
// KeyboardInput synthesizes the same indices from key presses.
class IInputSource {
public:
    virtual ~IInputSource() = default;

    // Returns the normalized [-1, 1] value for the given axis index.
    // Returns 0 if the axis index is not available.
    virtual float get_axis(int index) const = 0;

    // Returns true if the given button is currently pressed.
    virtual bool get_button(int index) const = 0;

    // Signal the input thread to stop. Called by Policy destructor.
    virtual void stop() = 0;
};

// Factory: tries to open the joystick at JOYSTICK_DEVICE env var
// (default: /dev/input/js2). Falls back to KeyboardInput if unavailable.
// Returns nullptr only when neither source is usable.
std::unique_ptr<IInputSource> create_input_source();
