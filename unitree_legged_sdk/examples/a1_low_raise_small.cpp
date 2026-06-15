/**********************************************************************
 * a1_low_raise_small.cpp
 *
 * Unitree A1 Low-level small raise test.
 *
 * Start condition:
 *   Robot should be in prone / low posture / supported body.
 *
 * Motion:
 *   1. Capture current q
 *   2. Hold current q
 *   3. Slightly extend legs
 *   4. Hold raised posture briefly
 *   5. Return to captured q
 *
 * Safety:
 *   - No absolute hard-coded standing pose
 *   - Uses current q as base
 *   - Small offsets only
 *   - No PositionProtect
 *   - Uses PositionLimit and PowerProtect
 *
 * Usage:
 *   sudo -E ./a1_low_raise_small
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

    void setMotor(int i, float q_des, float kp, float kd)
    {
        cmd.motorCmd[i].q = q_des;
        cmd.motorCmd[i].dq = 0.0f;
        cmd.motorCmd[i].Kp = kp;
        cmd.motorCmd[i].Kd = kd;
        cmd.motorCmd[i].tau = 0.0f;
    }

    void makeTarget()
    {
        for (int i = 0; i < 12; i++)
        {
            q_target[i] = q_start[i];
        }

        /*
         * Joint convention in this SDK:
         *   _0 : hip ab/ad
         *   _1 : hip pitch
         *   _2 : knee / calf
         *
         * From prone posture:
         *   knee q is around -2.7 rad.
         *   Increasing knee q toward -2.45 rad slightly extends the leg.
         *
         * This is intentionally small.
         */
        q_target[FR_1] = q_start[FR_1] - HIP_OFFSET;
        q_target[FL_1] = q_start[FL_1] - HIP_OFFSET;
        q_target[RR_1] = q_start[RR_1] - HIP_OFFSET;
        q_target[RL_1] = q_start[RL_1] - HIP_OFFSET;

        q_target[FR_2] = q_start[FR_2] + KNEE_OFFSET;
        q_target[FL_2] = q_start[FL_2] + KNEE_OFFSET;
        q_target[RR_2] = q_start[RR_2] + KNEE_OFFSET;
        q_target[RL_2] = q_start[RL_2] + KNEE_OFFSET;
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
                    q_start[i] = state.motorState[i].q;
                    q_des[i] = q_start[i];
                }

                makeTarget();

                captured = true;
                capture_time = motiontime;

                std::cout << "[RAISE_SMALL] captured current q" << std::endl;
                printAll();
            }

            /*
             * Before capture, use only weak damping.
             */
            for (int i = 0; i < 12; i++)
            {
                setMotor(i, state.motorState[i].q, 0.0f, 0.8f);
            }

            udp.SetSend(cmd);
            return;
        }

        int t = motiontime - capture_time;

        /*
         * Kp ramp:
         *   first 2 s: Kp 0 -> KP_HOLD
         */
        float kp_rate = smooth01(t / 1000.0f);
        float kp = KP_HOLD * kp_rate;
        float kd = KD_HOLD;

        /*
         * Motion schedule:
         *   0-2s    : hold captured q while Kp rises
         *   2-6s    : move to slightly raised posture
         *   6-8s    : hold raised posture
         *   8-12s   : return to captured q
         *   12s-    : finish
         */
        if (t < 1000)
        {
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = q_start[i];
            }
        }
        else if (t < 3000)
        {
            float r = smooth01((t - 1000) / 2000.0f);
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = lerp(q_start[i], q_target[i], r);
            }
        }
        else if (t < 4000)
        {
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = q_target[i];
            }
        }
        else if (t < 6000)
        {
            float r = smooth01((t - 4000) / 2000.0f);
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = lerp(q_target[i], q_start[i], r);
            }
        }
        else
        {
            for (int i = 0; i < 12; i++)
            {
                q_des[i] = q_start[i];
            }
        }

        for (int i = 0; i < 12; i++)
        {
            setMotor(i, q_des[i], kp, kd);
        }

        /*
         * Do not use PositionProtect here.
         * It triggers on the natural prone calf angle around -155 deg.
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
                << " hip_off=" << HIP_OFFSET
                << " knee_off=" << KNEE_OFFSET
                << std::endl;

            printShort();
        }

        if (captured && t > 6500)
        {
            std::cout << "[A1_LOW_RAISE_SMALL] finished" << std::endl;
            exit(0);
        }
    }

    void printMotor(const char* name, int id)
    {
        std::cout
            << std::setw(4) << name
            << " q=" << std::setw(11) << state.motorState[id].q
            << " q_des=" << std::setw(11) << q_des[id]
            << " q_start=" << std::setw(11) << q_start[id]
            << " q_target=" << std::setw(11) << q_target[id]
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

    float q_start[12] = {0.0f};
    float q_target[12] = {0.0f};
    float q_des[12] = {0.0f};

    const float HIP_OFFSET = 0.12f;
    const float KNEE_OFFSET = 0.60f;

    const float KP_HOLD = 45.0f;
    const float KD_HOLD = 2.0f;

    float dt = 0.002f;
};

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "LOW RAISE SMALL." << std::endl
              << "Start from prone / low posture / supported body." << std::endl
              << "This program slightly extends legs and returns." << std::endl
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
