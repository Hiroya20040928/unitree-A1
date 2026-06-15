/**********************************************************************
 * a1_height_loop.cpp
 *
 * Unitree A1 High-level height test using official LoopFunc style.
 *
 * Purpose:
 *   Reproduce the control structure of example_walk:
 *     - InitEnvironment()
 *     - LoopFunc udp_send
 *     - LoopFunc udp_recv
 *     - LoopFunc control_loop
 *
 * Usage:
 *   sudo -E ./a1_height_loop
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <iostream>
#include <unistd.h>
#include <cmath>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level)
        : safe(LeggedType::A1),
          udp(8080, "192.168.123.161", 8082, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH)
    {
        udp.InitCmdData(cmd);
    }

    void UDPRecv()
    {
        udp.Recv();
    }

    void UDPSend()
    {
        udp.Send();
    }

    void RobotControl()
    {
        motiontime++;

        udp.GetRecv(state);

        udp.InitCmdData(cmd);

        cmd.mode = 1;
        cmd.forwardSpeed = 0.0f;
        cmd.sideSpeed = 0.0f;
        cmd.rotateSpeed = 0.0f;
        cmd.footRaiseHeight = 0.0f;

        /*
         * 0-1s      : normal stand
         * 1-4s      : ramp down
         * 4-8s      : hold low height
         * 8-11s     : ramp up
         * 11s以降   : normal stand
         */
        if (motiontime < 500)
        {
            cmd.bodyHeight = 0.00f;
        }
        else if (motiontime < 2000)
        {
            float rate = (motiontime - 500) / 1500.0f;
            cmd.bodyHeight = -0.20f * rate;
        }
        else if (motiontime < 4000)
        {
            cmd.bodyHeight = -0.20f;
        }
        else if (motiontime < 5500)
        {
            float rate = (motiontime - 4000) / 1500.0f;
            cmd.bodyHeight = -0.20f * (1.0f - rate);
        }
        else
        {
            cmd.bodyHeight = 0.00f;
        }

        udp.SetSend(cmd);

        if (motiontime % 500 == 0)
        {
            std::cout
                << "t=" << motiontime * 2 << "ms "
                << "send(mode=" << static_cast<int>(cmd.mode)
                << ", h=" << cmd.bodyHeight
                << ", fwd=" << cmd.forwardSpeed
                << ", rot=" << cmd.rotateSpeed
                << ") "
                << "recv(mode=" << static_cast<int>(state.mode)
                << ", fwd=" << state.forwardSpeed
                << ", tick=" << state.tick
                << ")"
                << std::endl;
        }

        if (motiontime > 6500)
        {
            std::cout << "[A1_HEIGHT_LOOP] finished" << std::endl;
            exit(0);
        }
    }

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};

    int motiontime = 0;
    float dt = 0.002f;
};

int main(void)
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cin.ignore();

    Custom custom(HIGHLEVEL);

    InitEnvironment();

    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
