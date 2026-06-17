#include "ModelRegistry.h"
#include "Policy.h"
#include "RobotConfig.h"
#include "TaskRegistry.h"
#include "skills/HeadSkill.h"
#include "skills/LocomotionSkill.h"

// Velocity-tracking task on flat terrain for the Booster T1 23-DOF humanoid.
//
// A thin composite: make_config() builds the robot/scene config (shared with
// main for publishCommand and with the skills for obs/decode), then the policy
// is assembled from two skills:
//   - LocomotionSkill — the trained ONNX policy driving the 21 body joints.
//   - HeadSkill        — a scripted look pose driving the 2 head joints.
// The head skill is registered last so it wins on the head joints (indices 0,1).
//
// Joint ordering follows robot.joint_names (hardware/DDS/MuJoCo XML depth-first
// order). sim_joint_names is identical — the ONNX model uses this same layout.
class T1Velocity : public Policy {
    public:
        T1Velocity(const std::string& task_name,
                   const std::string& model_name = "",
                   const std::string& inference_backend = "onnx")
            : Policy(make_config(task_name, model_name, inference_backend)) {
            add_skill(std::make_unique<LocomotionSkill>(config_));
            add_skill(std::make_unique<HeadSkill>());
        }

    private:
        static TaskConfig make_config(const std::string& task_name,
                                      const std::string& model_name = "",
                                      const std::string& inference_backend = "onnx") {
            TaskConfig cfg;
            cfg.inference_backend = inference_backend;
            cfg.task_name    = task_name;
            cfg.model_name   = model_name;
            cfg.model_path   = model_name.empty()
                ? ModelRegistry::resolve(cfg.task_name).string()
                : ModelRegistry::resolve(cfg.task_name, model_name).string();
            cfg.policy_dt    = 0.02f;

            cfg.action_scale = {
                0.0f, 0.0f, 0.0f, 0.0f,                        // Left arm
                0.0f, 0.0f, 0.0f, 0.0f,                        // Right arm
                0.0f,                                              // Waist
                0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f,         // Left leg
                0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f,         // Right leg
            };

            // Action-to-joint mapping: 21 policy outputs → 23 hardware joints.
            // Head joints (indices 0,1: AAHead_yaw, Head_pitch) are excluded;
            // they are driven by HeadSkill.
            cfg.action_to_joint_idx.resize(cfg.action_scale.size());
            for (size_t a = 0; a < cfg.action_to_joint_idx.size(); a++)
                cfg.action_to_joint_idx[a] = a + 2;  // skip head indices 0,1

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
                0.2f, -1.35f, 0.0f, -0.5f,              // Left arm
                0.2f,  1.35f, 0.0f,  0.5f,              // Right arm
                0.0f,                                 // Waist
                -0.24f, 0.0f, 0.0f, 0.5f, -0.3f, 0.0f, // Left leg
                -0.24f, 0.0f, 0.0f, 0.5f, -0.3f, 0.0f, // Right leg
            };

            cfg.robot.joint_stiffness     = {
                4.0f,   4.0f,
                50.0f,  50.0f,  50.0f, 50.0f,
                50.0f,  50.0f,  50.0f, 50.0f,
                200.0f,
                200.0f, 200.0f, 200.0f, 200.0f, 50.0f, 50.0f,
                200.0f, 200.0f, 200.0f, 200.0f, 50.0f, 50.0f,
            };

            cfg.robot.joint_damping = {
                1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, 1.0f,
                5.0f,
                5.0f, 5.0f, 5.0f, 5.0f, 2.5f, 2.5f,
                5.0f, 5.0f, 5.0f, 5.0f, 2.5f, 2.5f,
            };

            cfg.robot.effort_limit = {
                7.0f,  7.0f,                                        // Head
                18.0f, 18.0f, 18.0f, 18.0f,                        // Left arm
                18.0f, 18.0f, 18.0f, 18.0f,                        // Right arm
                30.0f,                                              // Waist
                45.0f, 30.0f, 30.0f, 65.0f, 24.0f, 15.0f,         // Left leg
                45.0f, 30.0f, 30.0f, 65.0f, 24.0f, 15.0f,         // Right leg
            };

            // Reflected motor inertia per joint — matches colosseum actuators.py.
            cfg.robot.joint_armature = {
                0.0018000f, 0.0018000f,                                 // Head
                0.0282528f, 0.0282528f, 0.0282528f, 0.0282528f,       // Left arm
                0.0282528f, 0.0282528f, 0.0282528f, 0.0282528f,       // Right arm
                0.0478125f,                                             // Waist
                0.0523908f, 0.0478125f, 0.0478125f, 0.0636012f, 0.0679104f, 0.0679104f,  // Left leg
                0.0523908f, 0.0478125f, 0.0478125f, 0.0636012f, 0.0679104f, 0.0679104f,  // Right leg
            };

            // Coulomb friction loss per joint — matches colosseum actuators.py.
            cfg.robot.joint_frictionloss = {
                0.03f, 0.03f,
                0.03f, 0.03f, 0.03f, 0.03f,
                0.03f, 0.03f, 0.03f, 0.03f,
                0.03f,
                0.03f, 0.03f, 0.03f, 0.03f, 0.03f, 0.03f,
                0.03f, 0.03f, 0.03f, 0.03f, 0.03f, 0.03f,
            };

            // Mechanically coupled ankle pairs (crank mechanism).
            cfg.robot.parallel_joint_indices = {15, 16, 21, 22};

            // Foot collision capsule contact setup — mirrors FEET_ONLY_COLLISION in colosseum.
            // The base robot XML sets capsule collision geoms for the soles.
            // MujocoPortal applies these overrides after loading the MJCF.
            cfg.robot.foot_contact.geom_names = {
                "left_foot1_collision",  "left_foot2_collision",
                "left_foot3_collision",  "left_foot4_collision",
                "left_foot5_collision",
                "right_foot1_collision", "right_foot2_collision",
                "right_foot3_collision", "right_foot4_collision",
                "right_foot5_collision",
            };

            // ── Safe startup sequence ─────────────────────────────────────
            // Prepare gains match the run gains — the zeta=2 damping already
            // provides smooth, over-damped approach to the default pose.
            cfg.robot.prepare_state.duration_s    = 1.0f;
            cfg.robot.prepare_state.stiffness     = {
                40., 40.,
                40., 50., 20., 20,
                40., 50., 20., 20,
                350.,
                350., 350., 180., 350., 350., 350.,
                350., 350., 180., 350., 350., 350.,
            };
            cfg.robot.prepare_state.damping       = {
                0.65, 0.65,
                0.5, 1.5, 0.2, 0.2,
                0.5, 1.5, 0.2, 0.2,
                5.,
                7.5, 7.5, 3., 5.5, 5.0, 5.0,
                7.5, 7.5, 3., 5.5, 5.0, 5.0,
            };
            cfg.robot.prepare_state.joint_pos     = {
                0, 0,
                0.0, -1.4, 0, -0.,
                0.0, 1.4, 0, 0.,
                0.,
                -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,
                -0.1, 0.0, 0.0, 0.2, -0.1, 0.0
            };

            return cfg;
        }
};

REGISTER_TASK("t1-velocity", T1Velocity);
