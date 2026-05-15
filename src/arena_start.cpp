/**
 * DDS-triggered start daemon for arena and dds-debug services.
 *
 * Subscribes to rt/remote_controller_state via the Booster SDK.
 * Joystick combos:
 *   LB + RB + Start → systemctl start arena
 *   LB + RB + Y     → sudo systemctl start dds-debug
 *   LB + RB + X     → sudo systemctl stop dds-debug
 *
 * (arena_stop handles LB+RB+Back → stop arena.)
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
static constexpr int kDiagPeriodS = 30;

static std::atomic<bool> g_running{true};
static std::mutex g_mutex;
static booster_interface::msg::RemoteControllerState g_latest_msg;
static bool g_has_msg = false;
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

static void startArena() {
    std::cout << "[arena_start] *** START arena (LB + RB + Start) ***\n" << std::flush;
    int ret = system("systemctl start arena");
    if (ret == 0)
        std::cout << "[arena_start] arena service started.\n" << std::flush;
    else
        std::cerr << "[arena_start] Failed to start arena service (exit code "
                  << ret << ").\n" << std::flush;
}

static void startDebug() {
    std::cout << "[arena_start] *** START dds-debug (LB + RB + Y) ***\n" << std::flush;
    int ret = system("systemctl start dds-debug");
    if (ret == 0)
        std::cout << "[arena_start] dds-debug service started.\n" << std::flush;
    else
        std::cerr << "[arena_start] Failed to start dds-debug service (exit code "
                  << ret << ").\n" << std::flush;
}

static void stopDebug() {
    std::cout << "[arena_start] *** STOP dds-debug (LB + RB + X) ***\n" << std::flush;
    int ret = system("systemctl stop dds-debug 2>/dev/null");
    if (ret == 0)
        std::cout << "[arena_start] dds-debug service stopped.\n" << std::flush;
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "[arena_start] ================================================\n"
              << "[arena_start] Subscribing to " << kTopic << "\n"
              << "[arena_start] Combos:\n"
              << "[arena_start]   LB+RB+Start → start arena\n"
              << "[arena_start]   LB+RB+Y     → start dds-debug\n"
              << "[arena_start]   LB+RB+X     → stop dds-debug\n"
              << "[arena_start] ================================================\n"
              << std::flush;

    std::cout << "[arena_start] FASTRTPS_DEFAULT_PROFILES_FILE = "
              << (std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE") ?: "(not set)")
              << "\n" << std::flush;

    try {
        booster::robot::ChannelFactory::Instance()->Init(0);
        std::cout << "[arena_start] ChannelFactory::Init(0) OK\n" << std::flush;
    } catch (const std::exception& e) {
        std::cerr << "[arena_start] ChannelFactory::Init FAILED: " << e.what() << "\n"
                  << std::flush;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "[arena_start] ChannelFactory::Init FAILED (unknown exception)\n"
                  << std::flush;
        return EXIT_FAILURE;
    }

    booster::robot::ChannelSubscriber<booster_interface::msg::RemoteControllerState> sub(kTopic);
    sub.InitChannel(rcCallback);
    std::cout << "[arena_start] Subscriber created for " << kTopic << "\n"
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
                    std::cout << "[arena_start] *** FIRST RemoteControllerState received! ***"
                              << " event=" << msg.event()
                              << " lx=" << msg.lx() << " ly=" << msg.ly()
                              << " rx=" << msg.rx() << " ry=" << msg.ry()
                              << " a=" << msg.a() << " b=" << msg.b()
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
            if (lb && rb && msg.start()) startArena();
            if (lb && rb && msg.y())     startDebug();
            if (lb && rb && msg.x())     stopDebug();
        }

        // Periodic diagnostic: report if we're alive and still waiting
        auto now = std::chrono::steady_clock::now();
        if (now - last_diag >= std::chrono::seconds(kDiagPeriodS)) {
            long long elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
                now.time_since_epoch()).count();
            if (!first_msg_logged) {
                std::cout << "[arena_start] " << elapsed_s
                          << " — still waiting for first RC message on " << kTopic
                          << " (rcvd=" << g_msg_count << ")\n" << std::flush;
            } else {
                std::cout << "[arena_start] " << elapsed_s
                          << " — alive, rcvd=" << g_msg_count << " msgs\n" << std::flush;
            }
            last_diag = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[arena_start] Shut down.\n" << std::flush;
    return EXIT_SUCCESS;
}
