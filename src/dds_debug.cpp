/**
 * DDS diagnostic tool — mirrors the exact startup path of arena_main to debug
 * FastDDS communication issues (especially missing joystick/RC commands).
 *
 * Uses RobotPortal for LowState subscription and create_input_source() for
 * joystick/RC input — exactly like main.cpp.  Does NOT publish commands,
 * switch to kCustom, or move the robot.
 *
 * Usage:
 *   dds_debug
 *
 * Press Ctrl-C to exit.
 */

#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include "inputs/DDSRemoteInput.h"
#include "inputs/IInputSource.h"
#include "inputs/JoystickInput.h"
#include "inputs/KeyboardInput.h"
#include "portals/IPortal.h"
#include "portals/RobotPortal.h"

static constexpr int kDiagPeriodS = 5;

static std::atomic<bool> g_running{true};

static void signalHandler(int) {
    g_running = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static const char* inputSourceName(const IInputSource* src) {
    if (!src) return "(null)";
    if (dynamic_cast<const DDSRemoteInput*>(src))  return "DDSRemoteInput";
    if (dynamic_cast<const JoystickInput*>(src))   return "JoystickInput";
    if (dynamic_cast<const KeyboardInput*>(src))   return "KeyboardInput";
    return "(unknown)";
}

static const char* envOr(const char* var, const char* fallback) {
    const char* v = std::getenv(var);
    return v ? v : fallback;
}

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostic thread
// ─────────────────────────────────────────────────────────────────────────────

static uint64_t g_loop_count = 0;

static void diagnosticLoop(const IInputSource* input, const RobotPortal* portal) {
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(kDiagPeriodS));

        std::cout << "\n──── Diagnostics (t=" << kDiagPeriodS * (++g_loop_count)
                  << "s) ───────────────────────────────\n"
                  << "  Input source: " << inputSourceName(input) << "\n"
                  << "  Portal has state: " << (portal && portal->hasState() ? "YES" : "NO") << "\n";

        if (input) {
            std::cout << "  ── Input axes  ────────────────────────────────\n"
                      << "    [" << input->get_axis(0)  // lx
                      << ", " << input->get_axis(1)   // ly
                      << ", " << input->get_axis(2)   // lt
                      << ", " << input->get_axis(3)   // rx
                      << ", " << input->get_axis(4)   // ry
                      << ", " << input->get_axis(5)   // rt
                      << "]\n"
                      << "  ── Input buttons ─────────────────────────────\n"
                      << "    A:" << input->get_button(0)
                      << " B:" << input->get_button(1)
                      << " X:" << input->get_button(2)
                      << " Y:" << input->get_button(3)
                      << " LB:" << input->get_button(4)
                      << " RB:" << input->get_button(5)
                      << " Back:" << input->get_button(6)
                      << " Start:" << input->get_button(7) << "\n";
        }

        if (portal && portal->hasState()) {
            const auto& s = portal->getState();
            std::cout << "  ── RobotState ─────────────────────────────────\n"
                      << "    RPY:   [" << s.rpy[0] << ", " << s.rpy[1] << ", " << s.rpy[2] << "]\n"
                      << "    Gyro:  [" << s.gyro[0] << ", " << s.gyro[1] << ", " << s.gyro[2] << "]\n"
                      << "    Acc:   [" << s.acc[0] << ", " << s.acc[1] << ", " << s.acc[2] << "]\n"
                      << "    PGrav: [" << s.projected_gravity[0] << ", "
                      << s.projected_gravity[1] << ", " << s.projected_gravity[2] << "]\n";
        }

        std::cout << "─────────────────────────────────────────────────\n"
                  << std::flush;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "╔═══════════════════════════════════════════════════════════╗\n"
              << "║         DDS Debug Tool — mirrors arena_main setup         ║\n"
              << "║  Uses RobotPortal & create_input_source() like main.cpp    ║\n"
              << "║  NO commands are published.  Press Ctrl-C to exit.         ║\n"
              << "╚═══════════════════════════════════════════════════════════╝\n\n"
              << std::flush;

    // ── Environment diagnostics ───────────────────────────────────────────

    std::cout << "── Environment ───────────────────────────────────────────\n"
              << "  FASTRTPS_DEFAULT_PROFILES_FILE: " << envOr("FASTRTPS_DEFAULT_PROFILES_FILE", "(not set)") << "\n"
              << "  JOYSTICK_DEVICE:                " << envOr("JOYSTICK_DEVICE", "/dev/input/js2 (default)") << "\n"
              << "  MODELS_DIR:                     " << envOr("MODELS_DIR", "(not set)") << "\n"
              << "  SPQR_SOUNDS_PATH:               " << envOr("SPQR_SOUNDS_PATH", "(not set)") << "\n"
              << "  LD_LIBRARY_PATH:                " << envOr("LD_LIBRARY_PATH", "(not set)") << "\n"
              << "──────────────────────────────────────────────────────────\n\n"
              << std::flush;

    // ── Create RobotPortal (sets up DDS, LowState subscriber) ─────────────
    // This is identical to what main.cpp does for --backend booster.

    const float policy_dt = 0.02f;  // 50 Hz

    std::cout << "[init] Creating RobotPortal...\n" << std::flush;
    auto portal = std::make_unique<RobotPortal>(policy_dt);

    std::cout << "[init] Calling portal->initialize()...\n" << std::flush;
    portal->initialize();
    std::cout << "[init] RobotPortal initialized.\n" << std::flush;

    // Play startup sound
    {
        const char* sounds_path = std::getenv("SPQR_SOUNDS_PATH");
        if (sounds_path) {
            portal->setSoundsPath(sounds_path);
            portal->playSound("start.wav");
            std::cout << "[init] Played start.wav from " << sounds_path << "\n" << std::flush;
        } else {
            std::cout << "[init] SPQR_SOUNDS_PATH not set — skipping sound.\n" << std::flush;
        }
    }

    // ── Create input source (joystick → DDSRemote → Keyboard fallback) ──
    // This is exactly what the policy uses for velocity commands.
    // BUT: RobotPortal already called ChannelFactory::Init(0).
    //      DDSRemoteInput::pollLoop() also calls Init(0) — this may fail
    //      if the ChannelFactory is not idempotent, causing fallback to
    //      KeyboardInput even though DDS is available.

    std::cout << "[init] Calling create_input_source()...\n" << std::flush;
    auto input = create_input_source();
    std::cout << "[init] Input source created: " << inputSourceName(input.get()) << "\n\n"
              << std::flush;

    if (dynamic_cast<KeyboardInput*>(input.get()) && std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE")) {
        std::cout << "\n*** WARNING: DDS is configured but KeyboardInput was selected.\n"
                  << "    This means DDSRemoteInput failed to initialize — likely\n"
                  << "    because ChannelFactory::Init(0) was already called by\n"
                  << "    RobotPortal. Check the DDSRemoteInit logs above.\n\n"
                  << std::flush;
    }

    // ── Wait for first LowState ──────────────────────────────────────────

    std::cout << "[main] Waiting for first robot state..." << std::flush;
    int wait_count = 0;
    while (g_running && !portal->hasState()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (++wait_count % 50 == 0)
            std::cout << "\n  Still waiting... (" << wait_count/10 << "s)" << std::flush;
    }

    if (!g_running) {
        std::cout << "\n[main] Interrupted before first state.\n" << std::flush;
        input->stop();
        return EXIT_SUCCESS;
    }
    std::cout << "\n[main] First robot state received!\n\n" << std::flush;

    // ── Main loop — continuously print state ─────────────────────────────

    std::thread diag_thread(diagnosticLoop, input.get(), portal.get());

    int print_counter = 0;
    while (g_running && portal->shouldContinue()) {
        if (print_counter++ % 25 == 0) {  // every ~0.5s at 50ms sleep
            const auto& s = portal->getState();

            // Print joint positions (leg joints 11-22 are most interesting)
            const char* leg_names[] = {
                "LHP","LHR","LHY","LKN","LAP","LAR",
                "RHP","RHR","RHY","RKN","RAP","RAR"
            };
            std::cout << "\n── Joint State ───────────────────────────────────────\n";
            for (int i = 0; i < 12; i++) {
                int j = 11 + i;
                std::cout << "  " << leg_names[i]
                          << "  q=" << std::fixed << std::setprecision(4) << std::setw(8) << s.joint_pos[j]
                          << "  v=" << std::setw(8) << s.joint_vel[j] << "\n";
            }
            std::cout << "──────────────────────────────────────────────────────\n" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ── Shutdown ─────────────────────────────────────────────────────────

    std::cout << "\n[shutdown] Stopping input source...\n" << std::flush;
    input->stop();
    std::cout << "[shutdown] DDS debug tool exiting.\n" << std::flush;

    return EXIT_SUCCESS;
}
