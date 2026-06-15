/**********************************************************************
 * a1_pose_once.cpp
 *
 * Unitree A1 old HighCmd posture control.
 *
 * Confirmed environment:
 *   example_walk works with UDP local port 8080.
 *
 * Usage:
 *   sudo -E ./a1_pose_once crouch
 *   sudo -E ./a1_pose_once standup
 *   sudo -E ./a1_pose_once lowhold
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

class A1PoseOnce
{
public:
    A1PoseOnce()
        : safe(LeggedType::A1),
          udp(8080, "192.168.123.161", 8082, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH)
    {
        udp.InitCmdData(cmd);
    }

    void sendStandHeight(float body_height, int duration_ms)
    {
        const int dt_us = 2000;
        const int loops = duration_ms * 1000 / dt_us;

        for (int i = 0; i < loops && running; ++i)
        {
            udp.Recv();
            udp.GetRecv(state);

            udp.InitCmdData(cmd);

            cmd.mode = 1;
            cmd.forwardSpeed = 0.0f;
            cmd.sideSpeed = 0.0f;
            cmd.rotateSpeed = 0.0f;
            cmd.bodyHeight = body_height;
            cmd.footRaiseHeight = 0.0f;

            udp.SetSend(cmd);
            udp.Send();

            usleep(dt_us);
        }
    }

    void rampHeight(float start_h, float end_h, int duration_ms)
    {
        const int dt_us = 2000;
        const int loops = duration_ms * 1000 / dt_us;

        for (int i = 0; i < loops && running; ++i)
        {
            float a = static_cast<float>(i) / static_cast<float>(loops - 1);
            float h = start_h + (end_h - start_h) * a;
            sendStandHeight(h, 2);
        }
    }

    void crouch()
    {
        std::cout << "[A1] crouch start" << std::endl;

        rampHeight(0.00f, -0.18f, 1500);
        sendStandHeight(-0.18f, 3000);

        std::cout << "[A1] crouch end" << std::endl;
    }

    void lowhold()
    {
        std::cout << "[A1] lowhold start. Ctrl+C to stop." << std::endl;

        rampHeight(0.00f, -0.18f, 1500);

        while (running)
        {
            sendStandHeight(-0.18f, 100);
        }

        std::cout << "[A1] lowhold end" << std::endl;
    }

    void standup()
    {
        std::cout << "[A1] standup start" << std::endl;

        rampHeight(-0.18f, 0.00f, 1500);
        sendStandHeight(0.00f, 2000);

        std::cout << "[A1] standup end" << std::endl;
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
        std::cerr << "Usage: sudo -E ./a1_pose_once crouch" << std::endl;
        std::cerr << "       sudo -E ./a1_pose_once standup" << std::endl;
        std::cerr << "       sudo -E ./a1_pose_once lowhold" << std::endl;
        return 1;
    }

    std::string action = argv[1];

    A1PoseOnce controller;

    if (action == "crouch")
    {
        controller.crouch();
    }
    else if (action == "standup")
    {
        controller.standup();
    }
    else if (action == "lowhold")
    {
        controller.lowhold();
    }
    else
    {
        std::cerr << "Unknown action: " << action << std::endl;
        return 1;
    }

    return 0;
}
