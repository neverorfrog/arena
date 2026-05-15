#include <cmath>

#include "GaitPhaseCommand.h"
#include "ModelRegistry.h"
#include "ObservationSpec.h"
#include "Policy.h"
#include "RobotConfig.h"
#include "TaskRegistry.h"
#include "VelocityCommand.h"

// Observation layout contract for this task. Mirrors the training VelocityPolicy
// spec exactly. validate_size() is checked at the end of build_observation()
// in debug builds.
struct VelocityObservationSpec : ObservationSpec {
    std::vector<Component> components() const override {
        return {
            {"base_ang_vel",  3},
            {"proj_gravity",  3},
            {"joint_pos",    23},
            {"joint_vel",    23},
            {"last_action",  23},
            {"vel_command",   3},
            {"phase_command", 4}
        };
    }
};

// Velocity-tracking task on flat terrain for the Booster T1 23-DOF humanoid.
//
// Observation layout (82 elements, must match training VelocityPolicy):
//   [0:3]   base angular velocity (gyro, rad/s)
//   [3:6]   projected gravity in body frame (m/s²)
//   [6:29]  joint positions relative to default pose (rad)
//   [29:52] joint velocities (rad/s)
//   [52:75] last action (raw network output, before scaling)
//   [75:78] velocity command [vx, vy, omega]
//   [78:82] gait phase [cos(φ_L), cos(φ_R), sin(φ_L), sin(φ_R)]
//
// Action decoding (per joint i):
//   target[i] = net_out[i] * action_scale[i] + default_joint_pos[i]
//   where action_scale[i] = 0.25 (matches Python ACTION_SCALE in constants.py)
//
// Joint ordering follows robot.joint_names (hardware/DDS/MuJoCo XML depth-first order).
// robot.sim_joint_names is identical — the ONNX model uses this same layout.
class T1Velocity : public Policy {
    public:
        T1Velocity(const std::string& model_name = "",
                   const std::string& inference_backend = "onnx")
            : Policy(make_config(model_name, inference_backend)) {
            input_source_ = create_input_source();
        }

    protected:
        // Task-specific command. Scaled from input_source_ axes by update_input().
        // Limits (1 m/s, 1 m/s, 1 rad/s) match training VelocityCommandConfig.
        VelocityCommand vel_command_{VelocityCommandConfig{1.0f, 1.0f, 1.0f}};

        // Gait phase clock — matches training GaitPhaseCommand.
        GaitPhaseCommand gait_phase_{};

        // Heading lock state — mirrors training rel_heading_envs=1.0 play mode.
        // When the joystick yaw axis is near zero, lock to the current heading
        // and compute a corrective vyaw via a P-controller (stiffness=0.5 rad/s/rad,
        // matching heading_control_stiffness in the training env).
        float last_yaw_       = 0.0f;
        float heading_target_ = 0.0f;
        bool  heading_locked_ = false;

        static float wrap_to_pi(float a) {
            while (a >  M_PI) a -= 2.0f * M_PI;
            while (a < -M_PI) a += 2.0f * M_PI;
            return a;
        }

        // Read joystick/keyboard axes and convert to velocity command.
        // Axis mapping (Linux joystick numbering):
        //   axis 1 (left stick Y, negated) → vx
        //   axis 0 (left stick X, negated) → vy
        //   axis 3 (right stick X, negated) → vyaw
        //
        // When the yaw axis is near zero the heading is locked and vyaw becomes
        // a P-controller correction toward heading_target_ (same as training
        // heading mode with heading_control_stiffness=0.5).
        void update_input() override {
            if (!input_source_) return;
            // vel_command_.vx = -input_source_->get_axis(1) * vel_command_.vx_max;
            // vel_command_.vy = -input_source_->get_axis(0) * vel_command_.vy_max;
            // const float raw_yaw = -input_source_->get_axis(3);
            // if (std::abs(raw_yaw) < 0.05f) {
            //     if (!heading_locked_) {
            //         heading_locked_  = true;
            //         heading_target_ = last_yaw_;
            //     }
            //     const float err = wrap_to_pi(heading_target_ - last_yaw_);
            //     vel_command_.vyaw = std::clamp(0.5f * err, -vel_command_.vyaw_max, vel_command_.vyaw_max);
            // } else {
            //     heading_locked_ = false;
            //     heading_target_ = last_yaw_;
            //     vel_command_.vyaw = raw_yaw * vel_command_.vyaw_max;
            // }
            vel_command_.set_normalized(
                -input_source_->get_axis(1),
                -input_source_->get_axis(0),
                -input_source_->get_axis(3)
            );
        }

        void build_observation(const RobotState& state) override {
            static const VelocityObservationSpec spec;
            observation.clear();
            last_yaw_ = state.rpy[2];

            // 1. Base angular velocity (3)
            observation.push_back(state.gyro[0]);
            observation.push_back(state.gyro[1]);
            observation.push_back(state.gyro[2]);

            // 2. Projected gravity (3)
            observation.push_back(state.projected_gravity[0]);
            observation.push_back(state.projected_gravity[1]);
            observation.push_back(state.projected_gravity[2]);

            // 3. Joint positions relative to default (23, in sim order)
            // real2sim[i] = hardware index that feeds sim slot i.
            for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
                int h = robot_data_.real2sim[i];
                observation.push_back(state.joint_pos[h] - config_.robot.default_joint_pos[h]);
            }

            // 4. Joint velocities (23, in sim order)
            for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
                observation.push_back(state.joint_vel[robot_data_.real2sim[i]]);
            }

            // 5. Last action (23)
            for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
                observation.push_back(last_action[i]);
            }

            // 6. Velocity command (3) — rate-limited toward target
            vel_command_.step_filter(config_.policy_dt);
            observation.push_back(vel_command_.vx);
            observation.push_back(vel_command_.vy);
            observation.push_back(vel_command_.vyaw);

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
                // print leg joint obs (hardware indices 11-22 → obs indices [17..28])
                // joint_pos slots are obs indices [6..28], joint_vel are [29..51]
                const char* leg_names[] = {"LHP","LHR","LHY","LKN","LAP","LAR",
                                           "RHP","RHR","RHY","RKN","RAP","RAR"};
                std::cout << "  leg pos_rel: ";
                for (int k = 0; k < 12; k++)
                    std::cout << leg_names[k] << "=" << observation[6 + 11 + k] << " ";
                std::cout << "\n  leg vel:     ";
                for (int k = 0; k < 12; k++)
                    std::cout << leg_names[k] << "=" << observation[29 + 11 + k] << " ";
                std::cout << "\n  last_action: ";
                for (int k = 0; k < 12; k++)
                    std::cout << leg_names[k] << "=" << observation[52 + 11 + k] << " ";
                std::cout << "\n" << std::flush;
            }
            obs_diag++;

            // 7. Gait phase (4) — advance clock and push [cos(φ_L), cos(φ_R), sin(φ_L), sin(φ_R)]
            float horiz_speed = std::hypot(vel_command_.vx, vel_command_.vy);
            gait_phase_.advance(config_.policy_dt, horiz_speed);
            for (float v : gait_phase_.command()) observation.push_back(v);

#ifndef NDEBUG
            spec.validate_size(static_cast<int>(observation.size()));
#endif
        }

        static TaskConfig make_config(const std::string& model_name = "",
                                      const std::string& inference_backend = "onnx") {
            TaskConfig cfg;
            cfg.inference_backend = inference_backend;
            cfg.task_name    = "t1-velocity";
            cfg.model_name   = model_name;
            cfg.model_path   = model_name.empty()
                ? ModelRegistry::resolve(cfg.task_name).string()
                : ModelRegistry::resolve(cfg.task_name, model_name).string();
            cfg.policy_dt    = 0.02f;
            cfg.action_scale.fill(0.25f);
            // Arm joints in hardware order (indices 2-9): reduced to 0.01
            for (int i = 2; i <= 9; i++) cfg.action_scale[i] = 0.01f;

            // ── Scene (MujocoPortal) ──────────────────────────────────────────
            // PROJECT_ROOT is injected by CMake as the repository root.
            cfg.scene_mjcf_path = std::string(PROJECT_ROOT)
                + "/assets/scene.xml";

            // ── Robot hardware ────────────────────────────────────────────────
            cfg.robot.name      = "Booster_T1_23DOF";
            cfg.robot.mjcf_path = "/home/neverorfrog/code/spqr/colosseum/src/colosseum/robots/t1_23dof/xmls/T1_23dof.xml";

            // Hardware order = DDS JointIndex enum = MuJoCo XML depth-first.
            // sim_joint_names is identical — the ONNX model was trained with
            // this observation layout (no reordering needed).
            cfg.robot.joint_names = cfg.robot.sim_joint_names = {
                "AAHead_yaw",          "Head_pitch",
                "Left_Shoulder_Pitch", "Left_Shoulder_Roll",
                "Left_Elbow_Pitch",    "Left_Elbow_Yaw",
                "Right_Shoulder_Pitch","Right_Shoulder_Roll",
                "Right_Elbow_Pitch",   "Right_Elbow_Yaw",
                "Waist",
                "Left_Hip_Pitch",  "Left_Hip_Roll",  "Left_Hip_Yaw",
                "Left_Knee_Pitch", "Left_Ankle_Pitch","Left_Ankle_Roll",
                "Right_Hip_Pitch", "Right_Hip_Roll",  "Right_Hip_Yaw",
                "Right_Knee_Pitch","Right_Ankle_Pitch","Right_Ankle_Roll",
            };

            cfg.robot.default_joint_pos = {
                0.0f,  0.0f,                            // Head yaw, pitch
                0.2f, -1.3f, 0.0f, -0.5f,              // Left arm
                0.2f,  1.3f, 0.0f,  0.5f,              // Right arm
                0.0f,                                 // Waist
                -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f, // Left leg
                -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f, // Right leg
            };

            cfg.robot.joint_stiffness = {
                5.0f,   5.0f,                                       // Head
                20.0f,  20.0f,  20.0f, 20.0f,                      // Left arm
                20.0f,  20.0f,  20.0f, 20.0f,                      // Right arm
                150.0f,                                             // Waist
                200.0f, 200.0f, 200.0f, 200.0f, 50.0f, 50.0f,      // Left leg
                200.0f, 200.0f, 200.0f, 200.0f, 50.0f, 50.0f,      // Right leg
            };

            cfg.robot.joint_damping = {
                0.5f, 0.5f,
                2.0f, 2.0f, 2.0f, 2.0f,
                2.0f, 2.0f, 2.0f, 2.0f,
                5.0f,
                5.0f, 5.0f, 5.0f, 5.0f, 1.0f, 1.0f,
                5.0f, 5.0f, 5.0f, 5.0f, 1.0f, 1.0f,
            };

            cfg.robot.effort_limit = {
                7.0f,  7.0f,                                        // Head (neck)
                36.0f, 36.0f, 36.0f, 36.0f,                        // Left arm
                36.0f, 36.0f, 36.0f, 36.0f,                        // Right arm
                60.0f,                                              // Waist
                90.0f, 60.0f, 60.0f, 130.0f, 36.0f, 50.0f,        // Left leg
                90.0f, 60.0f, 60.0f, 130.0f, 36.0f, 50.0f,        // Right leg
            };

            // Reflected inertia per joint (rotor_inertia * 1e-6 * gear_ratio²).
            // Matches Python MotorSpec.reflected_inertia in actuators.py.
            cfg.robot.joint_armature = {
                0.0018f, 0.0018f,                                     // Neck
                0.0283f, 0.0283f, 0.0283f, 0.0283f,                  // Left arm
                0.0283f, 0.0283f, 0.0283f, 0.0283f,                  // Right arm
                0.0478f,                                              // Waist
                0.0524f, 0.0478f, 0.0478f, 0.0636f, 0.0340f, 0.0340f, // Left leg
                0.0524f, 0.0478f, 0.0478f, 0.0636f, 0.0340f, 0.0340f, // Right leg
            };

            // Mechanically coupled ankle pairs (crank mechanism).
            cfg.robot.parallel_joint_indices = {15, 16, 21, 22};

            // ── Safe startup sequence ─────────────────────────────────────
            // Prepare gains: stiff enough to hold pose, damped enough to
            // prevent oscillation. Ankle kd raised from 0.5 to 2.0.
            cfg.robot.prepare_state.duration_s    = 0.5f;
            cfg.robot.prepare_state.stiffness     = {
                5.0f,   5.0f,
                40.0f,  50.0f,  20.0f, 10.0f,
                40.0f,  50.0f,  20.0f, 10.0f,
                100.0f,
                200.0f, 200.0f, 100.0f, 200.0f, 100.0f, 100.0f,
                200.0f, 200.0f, 100.0f, 200.0f, 100.0f, 100.0f,
            };
            cfg.robot.prepare_state.damping       = {
                0.5f,  0.5f,
                1.0f,  2.0f, 0.5f, 0.5f,
                1.0f,  2.0f, 0.5f, 0.5f,
                5.0f,
                5.0f, 5.0f, 3.0f, 5.0f, 3.0f, 3.0f,
                5.0f, 5.0f, 3.0f, 5.0f, 3.0f, 3.0f,
            };
            cfg.robot.prepare_state.joint_pos     = {
                0.0f,  0.0f,
                0.2f, -1.3f, 0.0f, -0.5f,
                0.2f,  1.3f, 0.0f,  0.5f,
                0.0f,
                -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f,
                -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f,
            };

            return cfg;
        }
};

REGISTER_TASK("t1-velocity", T1Velocity);
