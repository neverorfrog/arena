/**
 * DDS-triggered start for arena_main.
 *
 * Subscribes to rt/remote_controller_state via the Booster SDK.
 * When LB + RB + Back are held simultaneously, starts the arena systemd service.
 *
 * Adapted from spqrbooster2026/src/app/start.cpp.
 */

#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

#include <booster/idl/b1/RemoteControllerState.h>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/channel/channel_factory.hpp>

static constexpr const char* kTopic = "rt/remote_controller_state";

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

static void startArena() {
    std::cout << "[arena_start] *** START (LB + RB + Back) ***\n" << std::flush;
    int ret = system("systemctl start arena");
    if (ret == 0)
        std::cout << "[arena_start] arena service started.\n" << std::flush;
    else
        std::cerr << "[arena_start] Failed to start arena service (exit code "
                  << ret << ").\n" << std::flush;
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "[arena_start] Subscribing to " << kTopic << "\n";
    std::cout << "[arena_start] Start combo: LB + RB + Back\n" << std::flush;

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
            startArena();

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[arena_start] Shut down.\n" << std::flush;
    return EXIT_SUCCESS;
}
