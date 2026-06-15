/**********************************************************************
 * a1_mode_test_8080.cpp
 *
 * Unitree A1 High-level mode test using confirmed UDP local port 8080.
 *
 * Usage:
 *   sudo -E ./a1_mode_test_8080 mode5
 *   sudo -E ./a1_mode_test_8080 mode6
 *   sudo -E ./a1_mode_test_8080 mode7
 *   sudo -E ./a1_mode_test_8080 stand
 *
 * Purpose:
 *   Check whether old A1 HighCmd accepts stand-down / damping modes
 *   when sent through the same UDP port as example_walk.
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <iostream>
#include <string>
#include <unistd.h>
#include <signal.h>

using namespace UNITREE_LEGGED_SDK;

static volatile bool running = true;

void onSignal(int)
{
    running = false;
}

class A1ModeTest8080
{
public:
    A1ModeTest8080()
        : safe(LeggedType::A1),
          udp(8080, "192.168.123.161", 8082, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH)
    {
        udp.InitCmdData(cmd);
    }

    void sendMode(unsigned char mode, int duration_ms)
    {
        const int dt_us = 2000;
        const int loops = duration_ms * 1000 / dt_us;

        for (int i = 0; i < loops && running; ++i)
        {
            udp.Recv();
            udp.GetRecv(state);

            udp.InitCmdData(cmd);

            cmd.mode = mode;
            cmd.forwardSpeed = 0.0f;
            cmd.sideSpeed = 0.0f;
            cmd.rotateSpeed = 0.0f;
            cmd.bodyHeight = 0.0f;
            cmd.footRaiseHeight = 0.0f;

            udp.SetSend(cmd);
            udp.Send();

            if (i % 500 == 0)
            {
                std::cout << "[A1_MODE_TEST_8080] send mode=" << static_cast<int>(mode)
                          << " recv.mode=" << static_cast<int>(state.mode)
                          << " fwd=" << state.forwardSpeed
                          << " tick=" << state.tick
                          << std::endl;
            }

            usleep(dt_us);
        }
    }

    void stand()
    {
        std::cout << "[A1_MODE_TEST_8080] stand mode=1 start" << std::endl;
        sendMode(1, 3000);
        std::cout << "[A1_MODE_TEST_8080] stand end" << std::endl;
    }

    void testMode(unsigned char mode)
    {
        std::cout << "[A1_MODE_TEST_8080] pre-stand mode=1" << std::endl;
        sendMode(1, 1500);

        std::cout << "[A1_MODE_TEST_8080] test mode=" << static_cast<int>(mode) << std::endl;
        sendMode(mode, 3000);

        std::cout << "[A1_MODE_TEST_8080] return stand mode=1" << std::endl;
        sendMode(1, 2000);

        std::cout << "[A1_MODE_TEST_8080] end" << std::endl;
    }

private:
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
};

int main(int argc, char** argv)
{
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    if (argc < 2)
    {
        std::cerr << "Usage: sudo -E ./a1_mode_test_8080 mode5" << std::endl;
        std::cerr << "       sudo -E ./a1_mode_test_8080 mode6" << std::endl;
        std::cerr << "       sudo -E ./a1_mode_test_8080 mode7" << std::endl;
        std::cerr << "       sudo -E ./a1_mode_test_8080 stand" << std::endl;
        return 1;
    }

    std::string action = argv[1];

    A1ModeTest8080 controller;

    if (action == "stand")
    {
        controller.stand();
    }
    else if (action == "mode5")
    {
        controller.testMode(5);
    }
    else if (action == "mode6")
    {
        controller.testMode(6);
    }
    else if (action == "mode7")
    {
        controller.testMode(7);
    }
    else
    {
        std::cerr << "Unknown action: " << action << std::endl;
        return 1;
    }

    return 0;
}
