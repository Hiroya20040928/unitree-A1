/**********************************************************************
 * a1_height_sweep_mode1.cpp
 *
 * Unitree A1 High-level bodyHeight sweep test.
 *
 * Safety:
 *   mode=1 only.
 *   No walking mode.
 *
 * Usage:
 *   sudo -E ./a1_height_sweep_mode1
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level)
        : safe(LeggedType::A1),
          udp(level)
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

    float getHeightCommand()
    {
        /*
         * motiontime unit:
         *   1 count = 2 ms
         */
        if (motiontime < 1000)       return 0.00f;   // 0-2s
        else if (motiontime < 2500)  return -0.10f;  // 2-5s
        else if (motiontime < 4000)  return -0.20f;  // 5-8s
        else if (motiontime < 5500)  return -0.30f;  // 8-11s
        else if (motiontime < 7000)  return 0.00f;   // 11-14s
        else if (motiontime < 8500)  return +0.10f;  // 14-17s
        else if (motiontime < 10000) return +0.20f;  // 17-20s
        else                         return 0.00f;   // 20s-
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
        cmd.bodyHeight = getHeightCommand();

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

        if (motiontime > 11500)
        {
            std::cout << "[A1_HEIGHT_SWEEP_MODE1] finished" << std::endl;
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
              << "WARNING: mode=1 only. No walking mode." << std::endl
              << "Height sweep: 0, -0.10, -0.20, -0.30, 0, +0.10, +0.20, 0" << std::endl
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
