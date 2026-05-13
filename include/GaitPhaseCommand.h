#pragma once
#include <array>
#include <cmath>
#include <random>

struct GaitPhaseCommandConfig {
    float gait_freq_lo         = 1.0f;   // Hz (matches colosseum cat_cfg.py)
    float gait_freq_hi         = 1.3f;   // Hz
    float speed_max            = 1.5f;   // m/s for freq scaling
    float gate_speed_threshold = 0.01f;  // m/s, gates on horizontal speed norm
};

// Two-foot gait phase clock. Mirrors colosseum GaitPhaseCommand.
//
// Phase convention: left starts at φ=0, right at φ=π (half-period offset).
// Phase wraps to [-π, π].
// Frequency is adaptive: freq = lo + (hi - lo) * clamp(|v_xy|/speed_max, 0, 1).
// When stopped (|v_xy| < threshold), phase is snapped to π (stance = 1),
// producing the observation [-1, -1, 0, 0].
struct GaitPhaseCommand {
    float freq_lo, freq_hi, speed_max, gate_speed_threshold;
    float phase_left  = 0.0f;
    float phase_right = static_cast<float>(M_PI);
    float freq        = 1.0f;

    explicit GaitPhaseCommand(GaitPhaseCommandConfig cfg = {})
        : freq_lo(cfg.gait_freq_lo)
        , freq_hi(cfg.gait_freq_hi)
        , speed_max(cfg.speed_max)
        , gate_speed_threshold(cfg.gate_speed_threshold) {}

    // Reset phases and sample a new frequency — call on episode reset.
    void resample(std::mt19937& rng) {
        std::uniform_real_distribution<float> dist(freq_lo, freq_hi);
        freq        = dist(rng);
        phase_left  = 0.0f;
        phase_right = static_cast<float>(M_PI);
    }

    // Advance clock by dt seconds, gated on horizontal speed.
    // Frequency is adaptive: freq = lo + (hi-lo) * clamp(|v_xy|/speed_max, 0, 1).
    // When stopped, snap to standing phase (π for both feet → [-1,-1,0,0]).
    void advance(float dt, float horizontal_speed) {
        constexpr float PI     = static_cast<float>(M_PI);
        constexpr float TWO_PI = 2.0f * PI;

        if (horizontal_speed <= gate_speed_threshold) {
            // Standing: snap both feet to π
            phase_left  = PI;
            phase_right = PI;
            return;
        }

        float t = horizontal_speed / speed_max;
        if (t > 1.0f) t = 1.0f;
        float adaptive_freq = freq_lo + (freq_hi - freq_lo) * t;
        float dphi = TWO_PI * dt * adaptive_freq;

        phase_left  = std::fmod(phase_left  + dphi + PI, TWO_PI) - PI;
        phase_right = std::fmod(phase_right + dphi + PI, TWO_PI) - PI;
    }

    // [cos(φ_L), cos(φ_R), sin(φ_L), sin(φ_R)] — the 4-element observation term.
    std::array<float, 4> command() const {
        return {
            std::cos(phase_left),  std::cos(phase_right),
            std::sin(phase_left),  std::sin(phase_right),
        };
    }

    // κ = (1 + cos(φ)) / 2 ∈ [0, 1] per foot. κ≈1 = stance, κ≈0 = swing.
    std::array<float, 2> stance_indicator() const {
        return {
            (1.0f + std::cos(phase_left))  / 2.0f,
            (1.0f + std::cos(phase_right)) / 2.0f,
        };
    }
};
