/**********************************************************************
 * a1_low_ready_cycle.cpp
 *
 * Unitree A1 Low-level ready posture cycle.
 *
 * Start condition:
 *   Robot should start from prone / low posture.
 *
 * Motion:
 *   1. Capture current prone q
 *   2. Hold captured q
 *   3. Raise to low-ready posture
 *   4. Hold low-ready posture
 *   5. Return to prone posture
 *
 * Safety:
 *   - No absolute hard-coded standing pose
 *   - Uses current q as base
 *   - No PositionProtect
 *   - Uses PositionLimit and PowerProtect
 *
 * Usage:
 *   sudo -E ./a1_low_ready_cycle
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

    float smooth01(float x)
    {
        x = clamp01(x);
        return x * x * (3.0f - 2.0f * x);
    }

    float lerp(float a, float b, float r)
    {
        return a + (b - a) * r;
    }

    void setMotor(int i, float q_desired, float kp, float kd)
    {
        cmd.motorCmd[i].q = q_desired;
        cmd.motorCmd[i].dq = 0.0f;
        cmd.motorCmd[i].Kp = kp;
        cmd.motorCmd[i].Kd = kd;
        cmd.motorCmd[i].tau = 0.0f;
    }

    void makeReadyTarget()
    {
        for (int i = 0; i < 12; i++)
        {
            q_ready[i] = q_prone[i];
        }

        /*
         * _0 : hip ab/ad
         * _1 : hip pitch
         * _2 : knee / calf
         *
         * From prone:
         *   hip pitch slightly decreases
         *   knee/calf increases from around -2.7 toward -1.8
         */
        q_ready[FR_1] = q_prone[FR_1] - HIP_OFFSET;
        q_ready[FL_1] = q_prone[FL_1] - HIP_OFFSET;
        q_ready[RR_1] = q_prone[RR_1] - HIP_OFFSET;
        q_ready[RL_1] = q_prone[RL_1] - HIP_OFFSET;

        q_ready[FR_2] = q_prone[FR_2] + KNEE_OFFSET;
        q_ready[FL_2] = q_prone[FL_2] + KNEE_OFFSET;
        q_ready[RR_2] = q_prone[RR_2] + KNEE_OFFSET;
        q_ready[RL_2] = q_prone[RL_2] + KNEE_OFFSET;
    }

    void RobotControl()
    {
        motiontime++;

        udp.GetRecv(state);
        udp.InitCmdData(cmd);

        if (!captured)
        {
            if (stateValid())
            {
                for (int i = 0; i < 12; i++)
                {
                    q_prone[i] = state.motorState[i].q;
                    q_des[i] = q_prone[i];
                }

                makeReadyTarget();

                captured = true;
                capture_time = motiontime;

                std::cout << "[READY_CYCLE] captured prone q" << std::endl;
                printAll();
            }

            for (int i = 0; i < 12; i++)
            {
                setMotor(i, state.motorState[i].q, 0.0f, 1.0f);
            }

            udp.SetSend(cmd);
            return;
        }

        int t = motiontime - capture_time;

        /*
         * Schedule at 500 Hz:
         *   0-2s    : hold prone, ramp gains
         *   2-7s    : prone -> ready
         *   7-12s   : hold ready
         *   12-17s  : ready -> prone
         *   17s-    : hold prone and finish
         */
        float kp_rate = smooth01(t / 1000.0f);
        float kp = KP_HOLD * kp_rate;
        float kd = KD_HOLD;

        if (t < 1000)
        {
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = q_prone[i];
            }
        }
        else if (t < 3500)
        {
            float r = smooth01((t - 1000) / 2500.0f);
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = lerp(q_prone[i], q_ready[i], r);
            }
        }
        else if (t < 6000)
        {
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = q_ready[i];
            }
        }
        else if (t < 8500)
        {
            float r = smooth01((t - 6000) / 2500.0f);
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = lerp(q_ready[i], q_prone[i], r);
            }
        }
        else
        {
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = q_prone[i];
            }
        }

        for (int i = 0; i < 12; i++)
        {
            setMotor(i, q_des[i], kp, kd);
        }

        /*
         * PositionProtect is intentionally disabled because prone calf angle
         * around -155 deg triggers SDK protection.
         */
        safe.PositionLimit(cmd);
        safe.PowerProtect(cmd, state, 1);

        udp.SetSend(cmd);

        if (motiontime % 500 == 0)
        {
            std::cout
                << "t=" << motiontime * 2 << "ms "
                << "phase_t=" << t * 2 << "ms "
                << "kp=" << kp
                << " kd=" << kd
                << " hip_off=" << HIP_OFFSET
                << " knee_off=" << KNEE_OFFSET
                << std::endl;

            printShort();
        }

        if (captured && t > 9500)
        {
            std::cout << "[A1_LOW_READY_CYCLE] finished" << std::endl;
            exit(0);
        }
    }

    void printMotor(const char* name, int id)
    {
        std::cout
            << std::setw(4) << name
            << " q=" << std::setw(11) << state.motorState[id].q
            << " q_des=" << std::setw(11) << q_des[id]
            << " q_prone=" << std::setw(11) << q_prone[id]
            << " q_ready=" << std::setw(11) << q_ready[id]
            << " dq=" << std::setw(11) << state.motorState[id].dq
            << " tau=" << std::setw(11) << state.motorState[id].tauEst
            << std::endl;
    }

    void printAll()
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

    void printShort()
    {
        printMotor("FR_1", FR_1);
        printMotor("FR_2", FR_2);
        printMotor("FL_1", FL_1);
        printMotor("FL_2", FL_2);
        printMotor("RR_1", RR_1);
        printMotor("RR_2", RR_2);
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

    float q_prone[12] = {0.0f};
    float q_ready[12] = {0.0f};
    float q_des[12] = {0.0f};

    /*
     * Tuned from successful clear-lift test.
     */
    const float HIP_OFFSET = 0.18f;
    const float KNEE_OFFSET = 0.90f;

    const float KP_HOLD = 55.0f;
    const float KD_HOLD = 2.5f;

    float dt = 0.002f;
};

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "LOW READY CYCLE." << std::endl
              << "Start from prone / low posture." << std::endl
              << "Motion: prone -> low-ready -> prone." << std::endl
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
