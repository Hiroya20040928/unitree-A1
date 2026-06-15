/**********************************************************************
 * a1_low_takeover_hold.cpp
 *
 * Unitree A1 Low-level takeover test.
 *
 * Purpose:
 *   Enter LOWLEVEL safely by capturing current joint angles,
 *   then holding those angles with gradually increasing Kp.
 *
 * Safety:
 *   - No walking command
 *   - No fixed absolute pose command at startup
 *   - Captures current q first
 *   - Ramps Kp from 0 to hold gain
 *
 * Usage:
 *   sudo -E ./a1_low_takeover_hold
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>

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

    bool stateValid()
    {
        float sum_abs_q = 0.0f;
        for (int i = 0; i < 12; i++)
        {
            sum_abs_q += std::fabs(state.motorState[i].q);
        }

        return sum_abs_q > 1.0f;
    }

    float clamp01(float x)
    {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }

    void setMotorHold(int i, float q_des, float kp, float kd)
    {
        cmd.motorCmd[i].q = q_des;
        cmd.motorCmd[i].dq = 0.0f;
        cmd.motorCmd[i].Kp = kp;
        cmd.motorCmd[i].Kd = kd;
        cmd.motorCmd[i].tau = 0.0f;
    }

    void RobotControl()
    {
        motiontime++;

        udp.GetRecv(state);
        udp.InitCmdData(cmd);

        /*
         * Phase 0:
         *   Until valid LowState arrives, send only weak damping.
         *   This phase should be very short.
         */
        if (!captured)
        {
            if (stateValid())
            {
                for (int i = 0; i < 12; i++)
                {
                    q_hold[i] = state.motorState[i].q;
                }

                captured = true;
                capture_time = motiontime;

                std::cout << "[TAKEOVER] captured current joint angles" << std::endl;
                printAllMotors();
            }

            for (int i = 0; i < 12; i++)
            {
                /*
                 * Before capture:
                 * q follows current measured q when valid.
                 * Kp=0 prevents position jump.
                 * Kd=1 gives weak damping only.
                 */
                float q_now = state.motorState[i].q;
                setMotorHold(i, q_now, 0.0f, 1.0f);
            }

            udp.SetSend(cmd);
            return;
        }

        /*
         * Phase 1:
         *   Hold captured pose.
         *   Ramp Kp slowly from 0 to KP_HOLD.
         */
        int t_after_capture = motiontime - capture_time;
        float rate = clamp01(t_after_capture / 1000.0f);  // 2 seconds at 500 Hz

        float kp = KP_HOLD * rate;
        float kd = KD_HOLD;

        for (int i = 0; i < 12; i++)
        {
            setMotorHold(i, q_hold[i], kp, kd);
        }

        /*
         * Use SDK safety filters.
         */
        if (motiontime > capture_time + 20)
        {
            safe.PositionLimit(cmd);
            safe.PowerProtect(cmd, state, 1);
            safe.PositionProtect(cmd, state, 0.20);
        }

        udp.SetSend(cmd);

        if (motiontime % 500 == 0)
        {
            std::cout
                << "t=" << motiontime * 2 << "ms "
                << "captured=" << captured
                << " kp=" << kp
                << " kd=" << kd
                << std::endl;

            printAllMotors();
        }

        if (captured && t_after_capture > 3500)
        {
            std::cout << "[A1_LOW_TAKEOVER_HOLD] finished" << std::endl;
            exit(0);
        }
    }

    void printMotor(const char* name, int id)
    {
        std::cout
            << std::setw(4) << name
            << " q="  << std::setw(11) << state.motorState[id].q
            << " q_hold=" << std::setw(11) << q_hold[id]
            << " dq=" << std::setw(11) << state.motorState[id].dq
            << " tau=" << std::setw(11) << state.motorState[id].tauEst
            << std::endl;
    }

    void printAllMotors()
    {
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

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};

    int motiontime = 0;
    int capture_time = 0;
    bool captured = false;

    float q_hold[12] = {0.0f};

    const float KP_HOLD = 18.0f;
    const float KD_HOLD = 1.2f;

    float dt = 0.002f;
};

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "LOW TAKEOVER HOLD." << std::endl
              << "Start from standby / low posture / supported body." << std::endl
              << "This program captures current q and holds it." << std::endl
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
