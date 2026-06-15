/**********************************************************************
 * a1_low_readonly.cpp
 *
 * Unitree A1 Low-level state reader.
 *
 * Safety:
 *   This program does NOT send motor position commands.
 *   It only receives and prints LowState motor angles.
 *
 * Usage:
 *   sudo -E ./a1_low_readonly
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

    void RobotControl()
    {
        motiontime++;
        udp.GetRecv(state);

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
            std::cout << "[A1_LOW_READONLY] finished" << std::endl;
            exit(0);
        }
    }

    void printMotor(const char* name, int id)
    {
        std::cout
            << std::setw(4) << name
            << " q="  << std::setw(10) << state.motorState[id].q
            << " dq=" << std::setw(10) << state.motorState[id].dq
            << " tau=" << std::setw(10) << state.motorState[id].tauEst
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
              << "READ ONLY. This program sends no motor command." << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cin.ignore();

    Custom custom(LOWLEVEL);

    InitEnvironment();

    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
