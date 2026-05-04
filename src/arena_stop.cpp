/**
 * DDS-triggered emergency stop for arena_main.
 *
 * Subscribes to rt/remote_controller_state via the Booster SDK.
 * When LB + RB + Start are held simultaneously, sends SIGTERM to arena_main.
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

static std::atomic<bool> g_running{true};
static std::mutex g_mutex;
static booster_interface::msg::RemoteControllerState g_latest_msg;
static bool g_has_msg = false;

static void signalHandler(int) {
    g_running = false;
}

static void rcCallback(const void* msg) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_latest_msg = *static_cast<const booster_interface::msg::RemoteControllerState*>(msg);
    g_has_msg = true;
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
    std::cerr << "[arena_stop] *** EMERGENCY STOP (LB + RB + Start) ***\n" << std::flush;
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

    std::cout << "[arena_stop] Subscribing to " << kTopic << "\n";
    std::cout << "[arena_stop] Emergency stop combo: LB + RB + Start\n" << std::flush;

    booster::robot::ChannelFactory::Instance()->Init(0);

    booster::robot::ChannelSubscriber<booster_interface::msg::RemoteControllerState> sub(kTopic);
    sub.InitChannel(rcCallback);

    while (g_running) {
        booster_interface::msg::RemoteControllerState msg;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            if (g_has_msg) {
                msg = g_latest_msg;
                g_has_msg = false;
            }
        }

        bool lb = msg.lb();
        bool rb = msg.rb();
        bool start = msg.start();
        if (lb && rb && start)
            emergencyStop();

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[arena_stop] Shut down.\n" << std::flush;
    return EXIT_SUCCESS;
}
