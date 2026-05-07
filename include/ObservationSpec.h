#pragma once
#include <stdexcept>
#include <string>
#include <vector>

// Describes the observation vector layout as a named list of components.
//
// Concrete specs are defined inline in each task's .cpp (e.g. VelocityObservationSpec
// in T1Velocity.cpp). The validate_size() method is called at the end of
// build_observation() under #ifndef NDEBUG to catch layout mismatches at
// startup rather than at inference time.
//
// Example:
//   struct MySpec : ObservationSpec {
//       std::vector<Component> components() const override {
//           return { {"gyro", 3}, {"joint_pos", 23} };
//       }
//   };
struct ObservationSpec {
    struct Component {
        std::string name;
        int size;
    };

    virtual ~ObservationSpec() = default;

    // Returns the ordered list of named components that make up the observation.
    virtual std::vector<Component> components() const = 0;

    // Total expected observation size (sum of component sizes).
    int total_size() const {
        int n = 0;
        for (const auto& c : components()) n += c.size;
        return n;
    }

    // Call at the end of build_observation() under #ifndef NDEBUG.
    // Throws std::runtime_error if actual != expected, with a human-readable
    // breakdown of which component would have been misaligned.
    void validate_size(int actual) const {
        int expected = total_size();
        if (actual == expected) return;

        std::string msg = "ObservationSpec size mismatch: got " + std::to_string(actual)
                        + ", expected " + std::to_string(expected) + "\nComponents:\n";
        int offset = 0;
        for (const auto& c : components()) {
            msg += "  [" + std::to_string(offset) + ":" + std::to_string(offset + c.size)
                 + "] " + c.name + " (" + std::to_string(c.size) + ")\n";
            offset += c.size;
        }
        throw std::runtime_error(msg);
    }
};
