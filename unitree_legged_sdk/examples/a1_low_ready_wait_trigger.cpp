/**********************************************************************
 * a1_low_ready_wait_trigger.cpp
 *
 * Unitree A1 Low-level ready-wait-trigger controller.
 *
 * Start condition:
 *   Robot should start from prone / low posture.
 *
 * Behavior:
 *   1. Capture prone q
 *   2. Raise to low-ready posture
 *   3. Hold low-ready posture indefinitely
 *   4. If /tmp/a1_choki_trigger exists:
 *        consume trigger
 *        move from ready to prone
 *        hold prone indefinitely
 *
 * This is the motion-control-side final test before adding vision.
 *
 * Usage:
 *   sudo -E ./a1_low_ready_wait_trigger
 *
 * Trigger from another SSH terminal:
 *   touch /tmp/a1_choki_trigger
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>
#include <sys/stat.h>
#include <cstdio>

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

    enum Phase
    {
        CAPTURE = 0,
        PRONE_HOLD_START,
        RAISE_TO_READY,
        READY_HOLD,
        LOWER_TO_PRONE,
        PRONE_HOLD_FINAL
    };

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

    bool triggerExists()
    {
        struct stat buffer;
        return stat(TRIGGER_PATH, &buffer) == 0;
    }

    void consumeTrigger()
    {
        std::remove(TRIGGER_PATH);
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
         * Tuned from successful ready-cycle test.
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

    void setDesiredArray(const float src[12])
    {
        for (int i = 0; i < 12; i++)
        {
            q_des[i] = src[i];
        }
    }

    void interpolateArray(const float a[12], const float b[12], float r)
    {
        for (int i = 0; i < 12; i++)
        {
            q_des[i] = lerp(a[i], b[i], r);
        }
    }

    void transitionTo(Phase next)
    {
        phase = next;
        phase_start_time = motiontime;

        std::cout << "[PHASE] -> " << phaseName(phase)
                  << " at t=" << motiontime * 2 << "ms"
                  << std::endl;
    }

    const char* phaseName(Phase p)
    {
        switch (p)
        {
            case CAPTURE: return "CAPTURE";
            case PRONE_HOLD_START: return "PRONE_HOLD_START";
            case RAISE_TO_READY: return "RAISE_TO_READY";
            case READY_HOLD: return "READY_HOLD";
            case LOWER_TO_PRONE: return "LOWER_TO_PRONE";
            case PRONE_HOLD_FINAL: return "PRONE_HOLD_FINAL";
            default: return "UNKNOWN";
        }
    }

    void RobotControl()
    {
        motiontime++;

        udp.GetRecv(state);
        udp.InitCmdData(cmd);

        if (phase == CAPTURE)
        {
            if (stateValid())
            {
                for (int i = 0; i < 12; i++)
                {
                    q_prone[i] = state.motorState[i].q;
                    q_des[i] = q_prone[i];
                }

                makeReadyTarget();

                std::cout << "[READY_WAIT_TRIGGER] captured prone q" << std::endl;
                printAll();

                transitionTo(PRONE_HOLD_START);
            }

            /*
             * Before valid state capture:
             * weak damping only.
             */
            for (int i = 0; i < 12; i++)
            {
                setMotor(i, state.motorState[i].q, 0.0f, 1.0f);
            }

            udp.SetSend(cmd);
            return;
        }

        int phase_t = motiontime - phase_start_time;

        /*
         * Kp ramp during initial takeover.
         */
        float global_t_after_capture = motiontime - phase_start_time;
        float kp = KP_HOLD;
        float kd = KD_HOLD;

        /*
         * Phase sequence:
         *   PRONE_HOLD_START : 0-2s
         *   RAISE_TO_READY   : 5s
         *   READY_HOLD       : indefinite, waits for trigger
         *   LOWER_TO_PRONE   : 4s
         *   PRONE_HOLD_FINAL : indefinite
         */
        if (phase == PRONE_HOLD_START)
        {
            float kp_rate = smooth01(phase_t / 1000.0f);  // 2 s
            kp = KP_HOLD * kp_rate;
            setDesiredArray(q_prone);

            if (phase_t > 1000)
            {
                transitionTo(RAISE_TO_READY);
            }
        }
        else if (phase == RAISE_TO_READY)
        {
            float r = smooth01(phase_t / 2500.0f);  // 5 s
            interpolateArray(q_prone, q_ready, r);

            if (phase_t > 2500)
            {
                transitionTo(READY_HOLD);
            }
        }
        else if (phase == READY_HOLD)
        {
            setDesiredArray(q_ready);

            if (triggerExists())
            {
                consumeTrigger();
                std::cout << "[TRIGGER] choki trigger consumed" << std::endl;
                transitionTo(LOWER_TO_PRONE);
            }
        }
        else if (phase == LOWER_TO_PRONE)
        {
            float r = smooth01(phase_t / 2000.0f);  // 4 s
            interpolateArray(q_ready, q_prone, r);

            if (phase_t > 2000)
            {
                transitionTo(PRONE_HOLD_FINAL);
            }
        }
        else if (phase == PRONE_HOLD_FINAL)
        {
            setDesiredArray(q_prone);
        }

        for (int i = 0; i < 12; i++)
        {
            setMotor(i, q_des[i], kp, kd);
        }

        /*
         * PositionProtect intentionally disabled because prone calf angle
         * around -155 deg triggers SDK protection.
         */
        safe.PositionLimit(cmd);
        safe.PowerProtect(cmd, state, 1);

        udp.SetSend(cmd);

        if (motiontime % 500 == 0)
        {
            std::cout
                << "t=" << motiontime * 2 << "ms "
                << "phase=" << phaseName(phase)
                << " phase_t=" << phase_t * 2 << "ms "
                << "kp=" << kp
                << " kd=" << kd
                << " trigger=" << (triggerExists() ? 1 : 0)
                << std::endl;

            printShort();
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
    int phase_start_time = 0;
    Phase phase = CAPTURE;

    float q_prone[12] = {0.0f};
    float q_ready[12] = {0.0f};
    float q_des[12] = {0.0f};

    const float HIP_OFFSET = 0.18f;
    const float KNEE_OFFSET = 0.90f;

    const float KP_HOLD = 55.0f;
    const float KD_HOLD = 2.5f;

    const char* TRIGGER_PATH = "/tmp/a1_choki_trigger";

    float dt = 0.002f;
};

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "LOW READY WAIT TRIGGER." << std::endl
              << "Start from prone / low posture." << std::endl
              << "Motion: prone -> ready -> wait -> trigger -> prone." << std::endl
              << "Trigger path: /tmp/a1_choki_trigger" << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cin.ignore();

    /*
     * Remove stale trigger before start.
     */
    std::remove("/tmp/a1_choki_trigger");

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
