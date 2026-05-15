/**
 * DDS-triggered emergency stop for arena_main.
 *
 * Subscribes to rt/remote_controller_state via the Booster SDK.
 * When LB + RB + Back are held simultaneously, sends SIGTERM to arena_main.
 *
 * Adapted from spqrbooster2026/src/app/stop.cpp.
 */

#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <booster/idl/b1/RemoteControllerState.h>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/channel/channel_factory.hpp>

static constexpr const char* kTopic = "rt/remote_controller_state";
static constexpr const char* kArenaProcess = "arena_main";
static constexpr int kDiagPeriodS = 30;

static std::atomic<bool> g_running{true};
static std::mutex g_mutex;
static booster_interface::msg::RemoteControllerState g_latest_msg;
static bool     g_has_msg = false;
static uint64_t g_msg_count = 0;

static void signalHandler(int) {
    g_running = false;
}

static void rcCallback(const void* msg) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_latest_msg = *static_cast<const booster_interface::msg::RemoteControllerState*>(msg);
    g_has_msg = true;
    g_msg_count++;
}

static pid_t findProcess(const char* name) {
    for (const auto& entry : std::filesystem::directory_iterator("/proc")) {
        if (!entry.is_directory())
            continue;
        const auto& fname = entry.path().filename().string();
        if (fname.empty() || !std::isdigit(fname[0]))
            continue;
        std::ifstream comm(entry.path() / "comm");
        std::string proc_name;
        if (comm >> proc_name && proc_name == name)
            return static_cast<pid_t>(std::stoi(fname));
    }
    return 0;
}

static void emergencyStop() {
    std::cerr << "[arena_stop] *** EMERGENCY STOP (LB + RB + Back) ***\n" << std::flush;
    pid_t pid = findProcess(kArenaProcess);
    if (pid > 0) {
        std::cerr << "[arena_stop] Sending SIGTERM to " << kArenaProcess
                  << " (pid " << pid << ")\n" << std::flush;
        kill(pid, SIGTERM);
    } else {
        std::cerr << "[arena_stop] " << kArenaProcess << " not found.\n" << std::flush;
    }
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "[arena_stop] ================================================\n"
              << "[arena_stop] Subscribing to " << kTopic << "\n"
              << "[arena_stop] Combo: LB+RB+Back → SIGTERM arena_main\n"
              << "[arena_stop] ================================================\n"
              << std::flush;

    std::cout << "[arena_stop] FASTRTPS_DEFAULT_PROFILES_FILE = "
              << (std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE") ?: "(not set)")
              << "\n" << std::flush;

    try {
        booster::robot::ChannelFactory::Instance()->Init(0);
        std::cout << "[arena_stop] ChannelFactory::Init(0) OK\n" << std::flush;
    } catch (const std::exception& e) {
        std::cerr << "[arena_stop] ChannelFactory::Init FAILED: " << e.what() << "\n"
                  << std::flush;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "[arena_stop] ChannelFactory::Init FAILED (unknown exception)\n"
                  << std::flush;
        return EXIT_FAILURE;
    }

    booster::robot::ChannelSubscriber<booster_interface::msg::RemoteControllerState> sub(kTopic);
    sub.InitChannel(rcCallback);
    std::cout << "[arena_stop] Subscriber created for " << kTopic << "\n"
              << std::flush;

    auto last_diag = std::chrono::steady_clock::now();
    bool first_msg_logged = false;

    while (g_running) {
        booster_interface::msg::RemoteControllerState msg;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            if (g_has_msg) {
                msg = g_latest_msg;
                g_has_msg = false;

                if (!first_msg_logged) {
                    std::cout << "[arena_stop] *** FIRST RemoteControllerState received! ***"
                              << " event=" << msg.event()
                              << " lx=" << msg.lx() << " ly=" << msg.ly()
                              << " lb=" << msg.lb() << " rb=" << msg.rb()
                              << " back=" << msg.back() << " start=" << msg.start()
                              << "\n" << std::flush;
                    first_msg_logged = true;
                }
            }
        }

        if (first_msg_logged) {
            bool lb = msg.lb();
            bool rb = msg.rb();
            bool back = msg.back();
            if (lb && rb && back)
                emergencyStop();
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_diag >= std::chrono::seconds(kDiagPeriodS)) {
            long long elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
                now.time_since_epoch()).count();
            if (!first_msg_logged) {
                std::cout << "[arena_stop] " << elapsed_s
                          << " — still waiting for first RC message on " << kTopic
                          << " (rcvd=" << g_msg_count << ")\n" << std::flush;
            } else {
                std::cout << "[arena_stop] " << elapsed_s
                          << " — alive, rcvd=" << g_msg_count << " msgs\n" << std::flush;
            }
            last_diag = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[arena_stop] Shut down.\n" << std::flush;
    return EXIT_SUCCESS;
}
