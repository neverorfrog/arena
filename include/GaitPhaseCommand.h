#pragma once
#include <array>
#include <cmath>
#include <random>

struct GaitPhaseCommandConfig {
    float gait_freq_lo         = 1.0f;   // Hz (matches colosseum cat_cfg.py)
    float gait_freq_hi         = 1.5f;   // Hz
    float speed_max            = 1.0f;   // m/s for freq scaling
    float gate_speed_threshold = 0.05f;  // m/s, gates on horizontal speed norm
};

// Two-foot gait phase clock. Mirrors colosseum GaitPhaseCommand.
//
// Phase convention: left starts at φ=0, right at φ=π (half-period offset).
// Phase wraps to [-π, π].
// Frequency is FIXED — sampled once at construction (midpoint of [lo,hi]) or via
// resample(). Matches training where each episode uses one constant sampled freq.
// When stopped (|v_xy| < threshold), phase is snapped to π (stance = 1),
// producing the observation [-1, -1, 0, 0].
// On the stand→walk transition the half-period offset is restored so the policy
// sees alternating phases instead of both feet in sync (which forces symmetric
// output and prevents real gait).
struct GaitPhaseCommand {
    float freq_lo, freq_hi, speed_max, gate_speed_threshold;
    float phase_left  = 0.0f;
    float phase_right = static_cast<float>(M_PI);
    float freq;

private:
    bool was_standing_ = false;

public:

    explicit GaitPhaseCommand(GaitPhaseCommandConfig cfg = {})
        : freq_lo(cfg.gait_freq_lo)
        , freq_hi(cfg.gait_freq_hi)
        , speed_max(cfg.speed_max)
        , gate_speed_threshold(cfg.gate_speed_threshold)
        , freq(0.5f * (cfg.gait_freq_lo + cfg.gait_freq_hi)) {}

    // Reset phases and sample a new frequency — call on episode reset.
    void resample(std::mt19937& rng) {
        std::uniform_real_distribution<float> dist(freq_lo, freq_hi);
        freq        = dist(rng);
        phase_left  = 0.0f;
        phase_right = static_cast<float>(M_PI);
        was_standing_ = false;
    }

    // Advance clock by dt seconds, gated on command speed.
    // Frequency is FIXED (sampled at resample()) to match training, where each
    // episode uses one constant frequency sampled from [freq_lo, freq_hi].
    // When stopped, snap to standing phase (π for both feet → [-1,-1,0,0]).
    void advance(float dt, float horizontal_speed) {
        constexpr float PI     = static_cast<float>(M_PI);
        constexpr float TWO_PI = 2.0f * PI;

        if (horizontal_speed <= gate_speed_threshold) {
            phase_left  = PI;
            phase_right = PI;
            was_standing_ = true;
            return;
        }

        // Stand→walk transition: restore half-period offset so phases are
        // never in sync. phase_left stays at π; phase_right jumps to 0.
        if (was_standing_) {
            phase_right  = std::fmod(phase_left + PI + PI, TWO_PI) - PI;  // = 0
            was_standing_ = false;
        }

        float dphi = TWO_PI * dt * freq;

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
