#include "skills/LocomotionSkill.h"

#include "ObservationSpec.h"
#include "engines/OnnxInferenceEngine.h"
#ifdef WITH_TENSORRT
#include "engines/TrtInferenceEngine.h"
#endif

#include <algorithm>
#include <cmath>
#include <iostream>

// Observation layout contract. Mirrors the training VelocityPolicy spec exactly.
// validate_size() is checked at the end of build_observation() in debug builds.
namespace {
struct VelocityObservationSpec : ObservationSpec {
    std::vector<Component> components() const override {
        return {
            {"base_ang_vel",  3},
            {"proj_gravity",  3},
            {"joint_pos",    23},
            {"joint_vel",    23},
            {"last_action",  21},
            {"vel_command",   3},
            {"phase_command", 4}
        };
    }
};

std::unique_ptr<IInferenceEngine> make_engine(const TaskConfig& cfg) {
    if (cfg.inference_backend == "trt") {
#ifdef WITH_TENSORRT
        std::string engine_path = cfg.model_path;
        auto pos = engine_path.rfind(".onnx");
        if (pos != std::string::npos)
            engine_path.replace(pos, 5, ".engine");
        return std::make_unique<TrtInferenceEngine>(engine_path);
#else
        throw std::runtime_error(
            "TensorRT backend not available (build without TensorRT)");
#endif
    }
    return std::make_unique<OnnxInferenceEngine>(cfg.model_path);
}
}  // namespace

LocomotionSkill::LocomotionSkill(const TaskConfig& cfg)
    : config_(cfg),
      robot_data_(cfg.robot),
      engine_(make_engine(cfg)),
      input_source_(create_input_source()) {
    observation_.reserve(engine_->input_dim());
}

LocomotionSkill::~LocomotionSkill() {
    if (input_source_) input_source_->stop();
}

void LocomotionSkill::reset() {
    last_action_.fill(0.0f);
}

float LocomotionSkill::wrap_to_pi(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// Read joystick/keyboard axes and convert to velocity command.
//   axis 1 (left stick Y, negated) → vx
//   axis 0 (left stick X, negated) → vy
//   axis 3 (right stick X, negated) → vyaw
// When the yaw axis is near zero the heading is locked and vyaw becomes a
// P-controller correction toward heading_target_ (training heading mode,
// heading_control_stiffness=0.5).
void LocomotionSkill::update_input() {
    if (!input_source_) return;
    vel_command_.vx = -input_source_->get_axis(1) * vel_command_.vx_max;
    vel_command_.vy = -input_source_->get_axis(0) * vel_command_.vy_max;
    const float raw_yaw = -input_source_->get_axis(3);
    if (std::abs(raw_yaw) < 0.05f) {
        if (!heading_locked_) {
            heading_locked_ = true;
            heading_target_ = last_yaw_;
        }
        const float err = wrap_to_pi(heading_target_ - last_yaw_);
        vel_command_.vyaw = std::clamp(0.5f * err, -vel_command_.vyaw_max, vel_command_.vyaw_max);
    } else {
        heading_locked_ = false;
        heading_target_ = last_yaw_;
        vel_command_.vyaw = raw_yaw * vel_command_.vyaw_max;
    }
}

void LocomotionSkill::build_observation(const RobotState& state) {
    static const VelocityObservationSpec spec;
    observation_.clear();
    last_yaw_ = state.rpy[2];

    // 1. Base angular velocity (3)
    observation_.push_back(state.gyro[0]);
    observation_.push_back(state.gyro[1]);
    observation_.push_back(state.gyro[2]);

    // 2. Projected gravity (3)
    observation_.push_back(state.projected_gravity[0]);
    observation_.push_back(state.projected_gravity[1]);
    observation_.push_back(state.projected_gravity[2]);

    // 3. Joint positions relative to default (23, in sim order)
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        int h = robot_data_.real2sim[i];
        observation_.push_back(state.joint_pos[h] - config_.robot.default_joint_pos[h]);
    }

    // 4. Joint velocities (23, in sim order)
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        observation_.push_back(state.joint_vel[robot_data_.real2sim[i]]);
    }

    // 5. Last action (21, raw network output)
    for (int i = 0; i < TaskConfig::NUM_ACTIONS; i++) {
        observation_.push_back(last_action_[i]);
    }

    // 6. Velocity command (3) — rate-limited toward target
    vel_command_.step_filter(config_.policy_dt);
    observation_.push_back(vel_command_.vx);
    observation_.push_back(vel_command_.vy);
    observation_.push_back(vel_command_.vyaw);

    static int obs_diag = 0;
    if (obs_diag % 50 == 0) {
        std::cout << "\n[vel] step=" << obs_diag
                  << " | cmd: vx=" << std::showpos << vel_command_.vx
                  << " vy=" << vel_command_.vy
                  << " wz=" << vel_command_.vyaw
                  << " | act: vx=" << state.base_lin_vel[0]
                  << " vy=" << state.base_lin_vel[1]
                  << " wz=" << state.gyro[2]
                  << " | err: vx=" << (vel_command_.vx - state.base_lin_vel[0])
                  << " vy=" << (vel_command_.vy - state.base_lin_vel[1])
                  << " wz=" << (vel_command_.vyaw - state.gyro[2])
                  << std::noshowpos << "\n";
        std::cout << "[OBS] step=" << obs_diag
                  << " cmd=[" << vel_command_.vx << "," << vel_command_.vy << "," << vel_command_.vyaw << "]"
                  << " gyro=[" << state.gyro[0] << "," << state.gyro[1] << "," << state.gyro[2] << "]"
                  << " pg=[" << state.projected_gravity[0] << "," << state.projected_gravity[1] << "," << state.projected_gravity[2] << "]\n";
        const char* leg_names[] = {"LHP","LHR","LHY","LKN","LAP","LAR",
                                   "RHP","RHR","RHY","RKN","RAP","RAR"};
        std::cout << "  leg pos_rel: ";
        for (int k = 0; k < 12; k++)
            std::cout << leg_names[k] << "=" << observation_[6 + 11 + k] << " ";
        std::cout << "\n  leg vel:     ";
        for (int k = 0; k < 12; k++)
            std::cout << leg_names[k] << "=" << observation_[29 + 11 + k] << " ";
        std::cout << "\n  last_action: ";
        for (int k = 0; k < 12; k++)
            std::cout << leg_names[k] << "=" << observation_[52 + 9 + k] << " ";
        std::cout << "\n" << std::flush;
    }
    obs_diag++;

    // 7. Gait phase (4) — advance clock and push [cos(φ_L), cos(φ_R), sin(φ_L), sin(φ_R)]
    float horiz_speed = std::hypot(vel_command_.vx, vel_command_.vy);
    gait_phase_.advance(config_.policy_dt, std::max(horiz_speed, std::abs(vel_command_.vyaw)));
    for (float v : gait_phase_.command()) observation_.push_back(v);

#ifndef NDEBUG
    spec.validate_size(static_cast<int>(observation_.size()));
#endif
}

void LocomotionSkill::compute(const RobotState& state,
                              std::array<float, TaskConfig::NUM_JOINTS>& targets) {
    update_input();
    build_observation(state);

    Eigen::VectorXf obs_vec =
        Eigen::Map<Eigen::VectorXf>(observation_.data(), observation_.size());
    Eigen::VectorXf action_vec = engine_->infer(obs_vec);

    if (config_.debug) {
        static int dbg_step = 0;
        if (dbg_step % 50 == 0) {
            float omin = 999, omax = -999;
            for (float v : observation_) { if (v < omin) omin = v; if (v > omax) omax = v; }
            std::cout << "obs[0-2]:    [" << observation_[0] << " " << observation_[1]
                      << " " << observation_[2] << "] (range " << omin << ".." << omax << ")\n";
            float amin = 999, amax = -999;
            for (int i = 0; i < TaskConfig::NUM_ACTIONS; i++) {
                if (action_vec[i] < amin) amin = action_vec[i];
                if (action_vec[i] > amax) amax = action_vec[i];
            }
            std::cout << "net_out:     " << amin << " .. " << amax << "\n" << std::flush;
        }
        dbg_step++;
    }

    // Store raw network output (21 values) and decode the 21 body joints.
    for (int a = 0; a < TaskConfig::NUM_ACTIONS; a++) {
        last_action_[a] = action_vec[a];
        int j = config_.action_to_joint_idx[a];  // a + 2 (head excluded)
        targets[j] = action_vec[a] * config_.action_scale[a]
                   + config_.robot.default_joint_pos[j];
    }
}
