#include "portals/CircusPortal.h"
#include "portals/IPortal.h"
#ifdef MUJOCO
#include "portals/MujocoPortal.h"
#endif
#include "portals/RobotPortal.h"
#include "TaskRegistry.h"
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>

static volatile std::sig_atomic_t terminated = 0;
static void signal_handler(int) { terminated = 1; }

// ──────────────────────────────────────────────────────────────────────────────
// main()
// ──────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Parse flags: --backend booster|mujoco|circus  --task <name>
    //              --inference onnx|trt
    //              --host <ip>  --port <n>  --robot <name>
    std::string backend    = "booster";
    std::string task_name  = "t1-velocity";
    std::string model_name;
    std::string inference_backend = "onnx";
    std::string circus_host = "127.0.0.1";
    int         circus_port = 5555;
    std::string circus_robot = "T1";
    for (int i = 0; i < argc; i++) {
        if (std::strcmp(argv[i], "--backend")   == 0 && i + 1 < argc) backend      = argv[++i];
        if (std::strcmp(argv[i], "--task")      == 0 && i + 1 < argc) task_name    = argv[++i];
        if (std::strcmp(argv[i], "--model")     == 0 && i + 1 < argc) model_name   = argv[++i];
        if (std::strcmp(argv[i], "--inference") == 0 && i + 1 < argc) inference_backend = argv[++i];
        if (std::strcmp(argv[i], "--host")      == 0 && i + 1 < argc) circus_host  = argv[++i];
        if (std::strcmp(argv[i], "--port")      == 0 && i + 1 < argc) circus_port  = std::stoi(argv[++i]);
        if (std::strcmp(argv[i], "--robot")     == 0 && i + 1 < argc) circus_robot = argv[++i];
    }

    // Create and initialize policy.
    auto policy = TaskRegistry::instance().create(task_name, model_name, inference_backend);
    const TaskConfig& cfg = policy->config();
    std::cout << "Task     : " << cfg.task_name << "\n"
              << "Model    : " << cfg.model_path << "\n"
              << "Backend  : " << backend << "\n"
              << "Inference: " << cfg.inference_backend << "\n" << std::flush;

    // Create portal.
    std::unique_ptr<IPortal> portal;
    if (backend == "mujoco") {
#ifdef MUJOCO
        portal = std::make_unique<MujocoPortal>(MujocoConfig{}, cfg);
#else
        std::cerr << "arena was built without MuJoCo/GLFW support. "
                     "Rebuild with mujoco and glfw in the pixi environment.\n";
        return -1;
#endif
    } else if (backend == "circus") {
        CircusConfig cc;
        cc.host       = circus_host;
        cc.port       = circus_port;
        cc.robot_name = circus_robot;
        portal = std::make_unique<CircusPortal>(cc, cfg);
    } else if (backend == "booster") {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        portal = std::make_unique<RobotPortal>(cfg.policy_dt);
    } else {
        std::cerr << "Unknown backend '" << backend
                  << "'. Use --backend booster|circus|mujoco\n";
        return -1;
    }

    std::cout << "DDS profile: " << (std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE") ?: "(not set)") << "\n" << std::flush;
    portal->initialize();
    std::cout << "Portal initialized, waiting for first state...\n" << std::flush;

    // Wait for first state.
    int wait_count = 0;
    while (!terminated && !portal->hasState()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (++wait_count % 50 == 0)
            std::cout << "Still waiting for state... (" << wait_count/10 << "s)\n" << std::flush;
    }
    if (terminated) return 0;

    // ── Activation (booster backend only) ─────────────────────────────────────
    if (backend == "booster") {
        auto* bp = dynamic_cast<RobotPortal*>(portal.get());
        if (bp) {
            const char* env = std::getenv("SPQR_SOUNDS_PATH");
            bp->setSoundsPath(env ? env : std::string(PROJECT_ROOT) + "/sounds");
            bp->playSound("start.wav");
            // Burst-hold at current position with prepare stiffness for 100ms.
            // This ensures the DDS buffer has a safe target when kCustom activates,
            // matching booster_deploy's hold+100ms sleep before mode switch.
            std::array<float, 23> hold_pos;
            const auto& s = portal->getState();
            for (int i = 0; i < 23; i++) hold_pos[i] = s.joint_pos[i];
            auto hold_start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - hold_start < std::chrono::milliseconds(100)) {
                bp->publishCommand(hold_pos.data(),
                    cfg.robot.prepare_state.stiffness.data(),
                    cfg.robot.prepare_state.damping.data());
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
            }
            // Switch to kCustom — firmware now reads our DDS commands.
            if (bp->changeMode(booster::robot::RobotMode::kCustom) != 0) {
                std::cerr << "Failed to switch robot to Custom mode.\n";
                return -1;
            }
            // Interpolate from current position to prepare pose with stiff gains.
            // booster_deploy does this AFTER kCustom at 500Hz for ~1s.
            bp->smoothPrepare(cfg.robot.prepare_state);
            // Hold at default pose with running gains for 0.1 seconds to let
            // joint velocities decay before the policy starts.
            // {
            //     auto settle_start = std::chrono::steady_clock::now();
            //     while (std::chrono::steady_clock::now() - settle_start
            //            < std::chrono::milliseconds(100)) {
            //         bp->publishCommand(cfg.robot.default_joint_pos.data(),
            //             cfg.robot.joint_stiffness.data(), cfg.robot.joint_damping.data());
            //         std::this_thread::sleep_for(std::chrono::microseconds(2000));
            //     }
            // }
            std::cout << "[Arena] Activated (custom mode).\n" << std::flush;
        }
    }

    // ── Main control loop ─────────────────────────────────────────────────────
    policy->reset();
    std::array<float, TaskConfig::NUM_JOINTS> targets{};

    while (!terminated && portal->shouldContinue()) {
        portal->updateState();
        targets = policy->get_action(portal->getState());
        portal->publishCommand(targets.data(),
            cfg.robot.joint_stiffness.data(), cfg.robot.joint_damping.data());
        portal->tick();
    }

    std::cout << "\nShutting down...\n" << std::flush;
    return 0;
}
