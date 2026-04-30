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
    std::string task_name  = "t1-velocity-flat";
    std::string inference_backend = "onnx";
    std::string circus_host = "127.0.0.1";
    int         circus_port = 5555;
    std::string circus_robot = "T1";
    for (int i = 0; i < argc; i++) {
        if (std::strcmp(argv[i], "--backend")   == 0 && i + 1 < argc) backend      = argv[++i];
        if (std::strcmp(argv[i], "--task")      == 0 && i + 1 < argc) task_name    = argv[++i];
        if (std::strcmp(argv[i], "--inference") == 0 && i + 1 < argc) inference_backend = argv[++i];
        if (std::strcmp(argv[i], "--host")      == 0 && i + 1 < argc) circus_host  = argv[++i];
        if (std::strcmp(argv[i], "--port")      == 0 && i + 1 < argc) circus_port  = std::stoi(argv[++i]);
        if (std::strcmp(argv[i], "--robot")     == 0 && i + 1 < argc) circus_robot = argv[++i];
    }

    // Create and initialize policy.
    auto policy = TaskRegistry::instance().create(task_name, inference_backend);
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
        std::cerr << "gladius was built without MuJoCo/GLFW support. "
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

    // Switch real robot to Custom mode (Booster only).
    if (backend == "booster") {
        // Move robot to safe prepare pose before starting the control loop.
        auto* booster_portal = dynamic_cast<RobotPortal*>(portal.get());
        booster_portal->prepare(cfg.robot.prepare_state);
        if (!booster_portal || booster_portal->changeMode(booster::robot::RobotMode::kCustom) != 0) {
            std::cerr << "Failed to switch robot to Custom mode.\n";
            return -1;
        }
    }
    // circus and mujoco backends need no mode switch.

    // Main control loop.
    policy->reset();
    std::array<float, TaskConfig::NUM_JOINTS> targets;

    while (!terminated && portal->shouldContinue()) {
        portal->updateState();
        targets = policy->get_action(portal->getState());
        portal->publishCommand(targets.data(), cfg.robot.joint_stiffness.data(), cfg.robot.joint_damping.data());
        portal->tick();
    }

    std::cout << "\nShutting down...\n" << std::flush;
    return 0;
}
