#include "portals/MujocoPortal.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

// ──────────────────────────────────────────────────────────────────────────────
// Construction / destruction
// ──────────────────────────────────────────────────────────────────────────────

MujocoPortal::MujocoPortal(MujocoConfig cfg, const TaskConfig& task_cfg)
    : cfg_(std::move(cfg)), task_cfg_(task_cfg) {}

MujocoPortal::~MujocoPortal() {
    if (window_) {
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
    if (mj_data_) mj_deleteData(mj_data_);
    if (mj_model_) mj_deleteModel(mj_model_);
}

// ──────────────────────────────────────────────────────────────────────────────
// initialize()
// ──────────────────────────────────────────────────────────────────────────────

void MujocoPortal::initialize() {
    char err[500] = "";

    // 1. Load scene-only XML, or compose scene+robot at runtime when requested.
    std::string load_path = task_cfg_.scene_mjcf_path;
    std::string temp_composed_path;
    if (!task_cfg_.robot.mjcf_path.empty()) {
        namespace fs = std::filesystem;
        const fs::path scene_abs = fs::absolute(task_cfg_.scene_mjcf_path);
        const fs::path robot_abs = fs::absolute(task_cfg_.robot.mjcf_path);

        // MuJoCo resolves nested resources (e.g., meshes) relative to the
        // top-level loaded XML, so compose next to the robot MJCF.
        const fs::path tmp_dir = robot_abs.parent_path();

        std::string temp_template = (tmp_dir / "arena_runtime_scene_XXXXXX").string();
        std::vector<char> temp_template_buf(temp_template.begin(), temp_template.end());
        temp_template_buf.push_back('\0');

        const int temp_fd = mkstemp(temp_template_buf.data());
        if (temp_fd < 0) {
            throw std::runtime_error(
                "MujocoPortal: failed to create temporary scene file in " + tmp_dir.string() +
                ": " + std::strerror(errno));
        }
        ::close(temp_fd);
        temp_composed_path = temp_template_buf.data();

        // MuJoCo resolves <include file="..."> relative to the composed XML directory.
        const fs::path composed_dir = fs::path(temp_composed_path).parent_path();
        const fs::path scene_rel = fs::relative(scene_abs, composed_dir);
        const fs::path robot_rel = fs::relative(robot_abs, composed_dir);

        std::ofstream composed(temp_composed_path, std::ios::out | std::ios::trunc);
        if (!composed.is_open()) {
            std::remove(temp_composed_path.c_str());
            throw std::runtime_error(
                "MujocoPortal: failed to open temporary scene file '" + temp_composed_path +
                "': " + std::strerror(errno));
        }
        composed << "<mujoco model=\"arena_runtime_scene\">\n"
                 << "  <include file=\"" << scene_rel.string() << "\"/>\n"
                 << "  <include file=\"" << robot_rel.string() << "\"/>\n"
                 << "</mujoco>\n";
        composed.close();
        load_path = temp_composed_path;
    }

    mj_model_ = mj_loadXML(load_path.c_str(), nullptr, err, sizeof(err));
    if (!temp_composed_path.empty()) {
        std::remove(temp_composed_path.c_str());
    }
    if (!mj_model_)
        throw std::runtime_error("MujocoPortal: mj_loadXML failed: " + std::string(err));

    mj_model_->opt.timestep = static_cast<double>(cfg_.physics_dt);
    mj_model_->opt.iterations = 10;
    mj_model_->opt.ls_iterations = 20;

    mj_data_ = mj_makeData(mj_model_);
    mj_resetData(mj_model_, mj_data_);

    // 2. Build joint index map.
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        int jid = mj_name2id(mj_model_, mjOBJ_JOINT, task_cfg_.robot.sim_joint_names[i].c_str());
        if (jid < 0)
            throw std::runtime_error("MujocoPortal: joint not found: " + task_cfg_.robot.sim_joint_names[i]);
        joint_qpos_idx_[i] = mj_model_->jnt_qposadr[jid];
        joint_dof_idx_[i]  = mj_model_->jnt_dofadr[jid];

        target_[i] = static_cast<double>(task_cfg_.robot.default_joint_pos[i]);
        kp_[i] = static_cast<double>(task_cfg_.robot.joint_stiffness[i]);
        kd_[i] = static_cast<double>(task_cfg_.robot.joint_damping[i]);
    }

    // 3. Apply per-joint physics overrides before mj_forward.
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        mj_model_->dof_armature[joint_dof_idx_[i]]    = static_cast<mjtNum>(task_cfg_.robot.joint_armature[i]);
        mj_model_->dof_frictionloss[joint_dof_idx_[i]] = static_cast<mjtNum>(task_cfg_.robot.joint_frictionloss[i]);
    }

    // 5. Resolve sensor offsets.
    auto sensor_offset = [&](const char* name) -> int {
        int sid = mj_name2id(mj_model_, mjOBJ_SENSOR, name);
        return sid >= 0 ? mj_model_->sensor_adr[sid] : -1;
    };

    // Keep compatibility with both naming schemes used in T1 XML variants.
    gyro_offset_ = sensor_offset("imu_ang_vel");
    if (gyro_offset_ < 0) gyro_offset_ = sensor_offset("angular-velocity");
    quat_offset_ = sensor_offset("orientation");

    // 6. Set initial pose: free-joint (3 pos + 4 quat) then per-joint positions.
    mj_data_->qpos[0] = 0.0; mj_data_->qpos[1] = 0.0;
    mj_data_->qpos[2] = static_cast<double>(cfg_.init_height);
    mj_data_->qpos[3] = 1.0; mj_data_->qpos[4] = 0.0;   // w, x
    mj_data_->qpos[5] = 0.0; mj_data_->qpos[6] = 0.0;   // y, z
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++)
        mj_data_->qpos[joint_qpos_idx_[i]] = task_cfg_.robot.default_joint_pos[i];
    mj_forward(mj_model_, mj_data_);

    // 5. Open viewer window.
    setupViewer();

    next_tick_ = Clock::now();
    updateState();
    has_state_ = true;
    std::cout << "[MujocoPortal] Ready. Policy dt=" << task_cfg_.policy_dt
              << "s, decimation=" << cfg_.decimation << "\n";
}

// ──────────────────────────────────────────────────────────────────────────────
// updateState()
// ──────────────────────────────────────────────────────────────────────────────

void MujocoPortal::updateState() {
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        state_.joint_pos[i] = static_cast<float>(mj_data_->qpos[joint_qpos_idx_[i]]);
        state_.joint_vel[i] = static_cast<float>(mj_data_->qvel[joint_dof_idx_[i]]);
    }

    if (gyro_offset_ >= 0) {
        for (int k = 0; k < 3; k++)
            state_.gyro[k] = static_cast<float>(mj_data_->sensordata[gyro_offset_ + k]);
    }

    if (quat_offset_ >= 0) {
        // MuJoCo framequat outputs (w,x,y,z).
        double q[4];
        for (int k = 0; k < 4; k++) {
            q[k] = mj_data_->sensordata[quat_offset_ + k];
            state_.root_quat[k] = static_cast<float>(q[k]);
        }
        auto pg = projectedGravity(q);
        for (int k = 0; k < 3; k++) state_.projected_gravity[k] = pg[k];

        // Compute body-frame linear velocity: v_body = q* * v_world
        float w = q[0], x = q[1], y = q[2], z = q[3];
        float vx = mj_data_->qvel[0], vy = mj_data_->qvel[1], vz = mj_data_->qvel[2];
        // quat_rotate_inverse: v' = v*(2w²-1) - 2w*cross(qv,v) + 2*qv*dot(qv,v)
        float w2 = w * w;
        float dot_qv_v = x*vx + y*vy + z*vz;
        float cx = y*vz - z*vy;
        float cy = z*vx - x*vz;
        float cz = x*vy - y*vx;
        state_.base_lin_vel[0] = vx * (2*w2 - 1) - 2*w*cx + 2*x*dot_qv_v;
        state_.base_lin_vel[1] = vy * (2*w2 - 1) - 2*w*cy + 2*y*dot_qv_v;
        state_.base_lin_vel[2] = vz * (2*w2 - 1) - 2*w*cz + 2*z*dot_qv_v;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// publishCommand()
// ──────────────────────────────────────────────────────────────────────────────

void MujocoPortal::publishCommand(const float* targets,
                                  const float* kp, const float* kd) {
    for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
        target_[i] = static_cast<double>(targets[i]);
        kp_[i] = static_cast<double>(kp[i]);
        kd_[i] = static_cast<double>(kd[i]);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// tick()  — step physics, render, sleep
// ──────────────────────────────────────────────────────────────────────────────

void MujocoPortal::tick() {
    // Apply explicit PD torques and step physics (decimation times).
    for (int s = 0; s < cfg_.decimation; s++) {
        // Clear external generalized forces before writing this substep.
        mju_zero(mj_data_->qfrc_applied, mj_model_->nv);
        for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
            const double q = mj_data_->qpos[joint_qpos_idx_[i]];
            const double dq = mj_data_->qvel[joint_dof_idx_[i]];
            const double tau_raw = kp_[i] * (target_[i] - q) - kd_[i] * dq;
            const double limit = static_cast<double>(task_cfg_.robot.effort_limit[i]);
            const double tau = std::clamp(tau_raw, -limit, limit);
            mj_data_->qfrc_applied[joint_dof_idx_[i]] = tau;
        }
        mj_step(mj_model_, mj_data_);
    }

    // Render if viewer is open.
    if (window_ && !glfwWindowShouldClose(window_)) {
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);

        // Track base position with camera.
        cam_.lookat[0] = mj_data_->qpos[0];
        cam_.lookat[1] = mj_data_->qpos[1];
        cam_.lookat[2] = mj_data_->qpos[2];

        mjv_updateScene(mj_model_, mj_data_, &opt_, nullptr, &cam_,
                        mjCAT_ALL, &scn_);
        mjrRect viewport{0, 0, width, height};
        mjr_render(viewport, &scn_, &con_);
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    // Sleep to maintain policy rate.
    next_tick_ += std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<float>(task_cfg_.policy_dt));
    std::this_thread::sleep_until(next_tick_);
}

// ──────────────────────────────────────────────────────────────────────────────
// shouldContinue()
// ──────────────────────────────────────────────────────────────────────────────

bool MujocoPortal::shouldContinue() const {
    if (!window_) return true;
    return !glfwWindowShouldClose(window_);
}

// ──────────────────────────────────────────────────────────────────────────────
// setupViewer()
// ──────────────────────────────────────────────────────────────────────────────

void MujocoPortal::setupViewer() {
    if (!glfwInit()) {
        std::cerr << "[MujocoPortal] GLFW init failed — running headless.\n";
        return;
    }
    window_ = glfwCreateWindow(1200, 900, "arena — MuJoCo", nullptr, nullptr);
    if (!window_) {
        std::cerr << "[MujocoPortal] Could not open window — running headless.\n";
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_makeScene(mj_model_, &scn_, 2000);
    mjr_defaultContext(&con_);
    mjr_makeContext(mj_model_, &con_, mjFONTSCALE_150);

    cam_.type     = mjCAMERA_FREE;
    cam_.distance = 3.0;
    cam_.elevation = -20.0;
}

// ──────────────────────────────────────────────────────────────────────────────
// projectedGravity()  — rotate [0,0,-1] into body frame using quaternion
// ──────────────────────────────────────────────────────────────────────────────

std::array<float, 3> MujocoPortal::projectedGravity(const double q[4]) {
    // q = (w, x, y, z)
    double w = q[0], x = q[1], y = q[2], z = q[3];
    return {
        static_cast<float>( 2.0*(w*y - x*z)),
        static_cast<float>(-2.0*(w*x + y*z)),
        static_cast<float>(-(1.0 - 2.0*(x*x + y*y)))
    };
}
