/**********************************************************************
 * a1_low_probe.cpp
 *
 * Unitree A1 Low-level probe.
 *
 * Safety:
 *   Sends zero-stiffness, zero-torque LowCmd.
 *   Does NOT command joint positions.
 *
 * Purpose:
 *   Make LOWLEVEL UDP exchange active and print LowState motor angles.
 *
 * Usage:
 *   sudo -E ./a1_low_probe
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <iostream>
#include <iomanip>
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

    void RobotControl()
    {
        motiontime++;

        udp.GetRecv(state);

        /*
         * Zero command:
         * - no position command
         * - no velocity command
         * - no torque command
         * - no stiffness
         *
         * This is only to keep UDP exchange alive.
         */
        udp.InitCmdData(cmd);

        for (int i = 0; i < 12; i++)
        {
            cmd.motorCmd[i].q = 0.0f;
            cmd.motorCmd[i].dq = 0.0f;
            cmd.motorCmd[i].Kp = 0.0f;
            cmd.motorCmd[i].Kd = 0.0f;
            cmd.motorCmd[i].tau = 0.0f;
        }

        udp.SetSend(cmd);

        if (motiontime % 500 == 0)
        {
            std::cout << "t=" << motiontime * 2 << "ms" << std::endl;

            printMotor("FR_0", FR_0);
            printMotor("FR_1", FR_1);
            printMotor("FR_2", FR_2);

            printMotor("FL_0", FL_0);
            printMotor("FL_1", FL_1);
            printMotor("FL_2", FL_2);

            printMotor("RR_0", RR_0);
            printMotor("RR_1", RR_1);
            printMotor("RR_2", RR_2);

            printMotor("RL_0", RL_0);
            printMotor("RL_1", RL_1);
            printMotor("RL_2", RL_2);

            std::cout << "--------------------------------" << std::endl;
        }

        if (motiontime > 5000)
        {
            std::cout << "[A1_LOW_PROBE] finished" << std::endl;
            exit(0);
        }
    }

    void printMotor(const char* name, int id)
    {
        std::cout
            << std::setw(4) << name
            << " q="  << std::setw(11) << state.motorState[id].q
            << " dq=" << std::setw(11) << state.motorState[id].dq
            << " tau=" << std::setw(11) << state.motorState[id].tauEst
            << std::endl;
    }

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};

    int motiontime = 0;
    float dt = 0.002f;
};

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "LOW PROBE: zero Kp, zero Kd, zero torque." << std::endl
              << "This should not command joint motion." << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cin.ignore();

    Custom custom(LOWLEVEL);

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
