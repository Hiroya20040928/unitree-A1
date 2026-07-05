/**********************************************************************
 * a1_low_action_safety_fsm.cpp
 *
 * Unitree A1 low-level action FSM with conservative safety supervisor.
 *
 * Based on the locally provided Unitree legged SDK interface:
 *   - UDP(level) native constructor
 *   - LowState: IMU, motorState[12], footForce[4], tick
 *   - Safety::PositionLimit / PowerProtect
 *
 * Start condition:
 *   Robot starts from prone / low posture.
 *
 * Action input:
 *   /tmp/a1_action
 *     ready
 *     prone
 *     choki
 *     emergency_prone
 *     stop
 *     wave
 *     shake
 *     sway
 *
 * Legacy input:
 *   /tmp/a1_choki_trigger also maps to prone.
 *
 * Usage:
 *   sudo -E ./a1_low_action_safety_fsm
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
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

    enum Phase
    {
        CAPTURE = 0,
        PRONE_HOLD_START,
        RAISE_TO_READY,
        READY_HOLD,
        LOWER_TO_PRONE,
        PRONE_HOLD,
        WAVE_UP,
        WAVE_OSC,
        WAVE_RETURN,
        SHAKE_UP,
        SHAKE_OSC,
        SHAKE_RETURN,
        SWAY_OSC,
        SWAY_RETURN,
        SAFETY_FREEZE,
        SAFETY_TO_PRONE
    };

    void UDPRecv()
    {
        udp.Recv();
    }

    void UDPSend()
    {
        udp.Send();
    }

    float clamp(float x, float lo, float hi)
    {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }

    float clamp01(float x)
    {
        return clamp(x, 0.0f, 1.0f);
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

    bool fileExists(const char* path)
    {
        struct stat buffer;
        return stat(path, &buffer) == 0;
    }

    bool readFirstLine(const char* path, std::string& out)
    {
        std::ifstream ifs(path);
        if (!ifs.good()) return false;
        std::getline(ifs, out);
        return true;
    }

    std::string trim(const std::string& s)
    {
        size_t a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) return "";
        size_t b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    }

    std::string readAction()
    {
        std::string action;

        if (readFirstLine(ACTION_PATH, action))
        {
            std::remove(ACTION_PATH);
            action = trim(action);
            if (!action.empty()) return action;
        }

        if (fileExists(CHOKI_TRIGGER_PATH))
        {
            std::remove(CHOKI_TRIGGER_PATH);
            return "choki";
        }

        return "";
    }

    bool stateValidBasic()
    {
        float sum_abs_q = 0.0f;
        for (int i = 0; i < 12; i++)
        {
            sum_abs_q += std::fabs(state.motorState[i].q);
        }
        return sum_abs_q > 1.0f;
    }

    int countContacts(float threshold)
    {
        int c = 0;
        for (int i = 0; i < 4; i++)
        {
            if (state.footForce[i] > threshold) c++;
        }
        return c;
    }

    bool footForceLooksValid()
    {
        int sum = 0;
        for (int i = 0; i < 4; i++) sum += std::max(0, static_cast<int>(state.footForce[i]));
        return sum > 20;
    }

    float maxAbsGyro()
    {
        float m = 0.0f;
        for (int i = 0; i < 3; i++) m = std::max(m, std::fabs(state.imu.gyroscope[i]));
        return m;
    }

    float maxAbsDq()
    {
        float m = 0.0f;
        for (int i = 0; i < 12; i++) m = std::max(m, std::fabs(state.motorState[i].dq));
        return m;
    }

    float maxAbsQError()
    {
        float m = 0.0f;
        for (int i = 0; i < 12; i++)
        {
            m = std::max(m, std::fabs(state.motorState[i].q - q_des[i]));
        }
        return m;
    }

    bool jointActualInsideLooseLimits()
    {
        for (int leg = 0; leg < 4; leg++)
        {
            int j0 = leg * 3 + 0;
            int j1 = leg * 3 + 1;
            int j2 = leg * 3 + 2;

            float hip   = state.motorState[j0].q;
            float thigh = state.motorState[j1].q;
            float calf  = state.motorState[j2].q;

            if (hip   < -0.95f || hip   > 0.95f) return false;
            if (thigh < -1.20f || thigh > 4.35f) return false;
            if (calf  < -2.85f || calf  > -0.75f) return false;
        }
        return true;
    }

    bool isOneLegActionPhase()
    {
        return phase == WAVE_UP || phase == WAVE_OSC || phase == WAVE_RETURN ||
               phase == SHAKE_UP || phase == SHAKE_OSC || phase == SHAKE_RETURN;
    }

    void updateCommunicationMonitor()
    {
        if (state.tick == 0)
        {
            same_tick_count = 0;
            return;
        }

        if (!tick_initialized)
        {
            last_tick = state.tick;
            same_tick_count = 0;
            tick_initialized = true;
            return;
        }

        if (state.tick == last_tick)
        {
            same_tick_count++;
        }
        else
        {
            same_tick_count = 0;
            last_tick = state.tick;
        }
    }

    bool safetyFault(std::string& reason)
    {
        if (phase == CAPTURE) return false;
        if (phase == SAFETY_FREEZE || phase == SAFETY_TO_PRONE) return false;

        if (!stateValidBasic())
        {
            reason = "invalid_low_state";
            return true;
        }

        if (same_tick_count > MAX_SAME_TICK_COUNT)
        {
            std::ostringstream oss;
            oss << "low_state_tick_stale count=" << same_tick_count;
            reason = oss.str();
            return true;
        }

        // The SDK comment says IMU attitude can drift under acceleration.
        // Therefore this is a conservative guard, not a proof of balance.
        float roll_deg  = state.imu.rpy[0] * RAD2DEG;
        float pitch_deg = state.imu.rpy[1] * RAD2DEG;

        if (std::fabs(roll_deg) > MAX_ROLL_DEG)
        {
            std::ostringstream oss;
            oss << "roll_over " << roll_deg;
            reason = oss.str();
            return true;
        }

        if (std::fabs(pitch_deg) > MAX_PITCH_DEG)
        {
            std::ostringstream oss;
            oss << "pitch_over " << pitch_deg;
            reason = oss.str();
            return true;
        }

        float g = maxAbsGyro();
        if (g > MAX_GYRO_RAD_S)
        {
            std::ostringstream oss;
            oss << "gyro_over " << g;
            reason = oss.str();
            return true;
        }

        float dq = maxAbsDq();
        if (dq > MAX_DQ_RAD_S)
        {
            std::ostringstream oss;
            oss << "dq_over " << dq;
            reason = oss.str();
            return true;
        }

        if (!jointActualInsideLooseLimits())
        {
            reason = "joint_actual_outside_loose_limit";
            return true;
        }

        // If the actual robot is far from commanded q, the foot may have slipped,
        // the body may be blocked, or the action may be too aggressive.
        float qe = maxAbsQError();
        if (qe > MAX_Q_ERROR_RAD)
        {
            std::ostringstream oss;
            oss << "q_error_over " << qe;
            reason = oss.str();
            return true;
        }

        // Use footForce only when it looks nonzero/usable. Some A1 setups report
        // weak or zero values, so do not globally require it.
        if (isOneLegActionPhase() && footForceLooksValid())
        {
            int contacts = countContacts(FOOT_FORCE_CONTACT_TH);
            if (contacts < 3)
            {
                std::ostringstream oss;
                oss << "support_contacts_less_than_3 contacts=" << contacts;
                reason = oss.str();
                return true;
            }
        }

        return false;
    }

    void captureCurrentTo(float dst[12])
    {
        for (int i = 0; i < 12; i++) dst[i] = state.motorState[i].q;
    }

    void copyArray(const float src[12], float dst[12])
    {
        for (int i = 0; i < 12; i++) dst[i] = src[i];
    }

    void setDesiredArray(const float src[12])
    {
        copyArray(src, q_des);
    }

    void interpolateArray(const float a[12], const float b[12], float r)
    {
        r = smooth01(r);
        for (int i = 0; i < 12; i++) q_des[i] = lerp(a[i], b[i], r);
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
        copyArray(q_prone, q_ready);

        q_ready[FR_1] = q_prone[FR_1] - HIP_OFFSET_READY;
        q_ready[FL_1] = q_prone[FL_1] - HIP_OFFSET_READY;
        q_ready[RR_1] = q_prone[RR_1] - HIP_OFFSET_READY;
        q_ready[RL_1] = q_prone[RL_1] - HIP_OFFSET_READY;

        q_ready[FR_2] = q_prone[FR_2] + KNEE_OFFSET_READY;
        q_ready[FL_2] = q_prone[FL_2] + KNEE_OFFSET_READY;
        q_ready[RR_2] = q_prone[RR_2] + KNEE_OFFSET_READY;
        q_ready[RL_2] = q_prone[RL_2] + KNEE_OFFSET_READY;
    }

    void makeWaveTargets()
    {
        copyArray(q_ready, q_wave_up);
        copyArray(q_ready, q_wave_down);

        // Conservative first version: front-right paw small lift/wave.
        // These offsets are intentionally small. Increase only after suspension tests.
        q_wave_up[FR_1]   = q_ready[FR_1] - 0.16f;
        q_wave_up[FR_2]   = q_ready[FR_2] - 0.20f;
        q_wave_down[FR_1] = q_ready[FR_1] - 0.05f;
        q_wave_down[FR_2] = q_ready[FR_2] - 0.08f;

        // Slightly stiffen the other three support legs by not moving them.
    }

    void makeShakeTargets()
    {
        copyArray(q_ready, q_shake_up);
        copyArray(q_ready, q_shake_down);

        // Paw-like up/down. Smaller than wave.
        q_shake_up[FR_1]   = q_ready[FR_1] - 0.12f;
        q_shake_up[FR_2]   = q_ready[FR_2] - 0.16f;
        q_shake_down[FR_1] = q_ready[FR_1] - 0.02f;
        q_shake_down[FR_2] = q_ready[FR_2] - 0.05f;
    }

    void makeSwayTarget(float dst[12], float s)
    {
        copyArray(q_ready, dst);

        // Very small lateral hip-ab/ad animation around ready.
        // This is not dynamic balance control; it is only a low-amplitude display motion.
        dst[FR_0] = q_ready[FR_0] + 0.05f * s;
        dst[RR_0] = q_ready[RR_0] + 0.05f * s;
        dst[FL_0] = q_ready[FL_0] - 0.05f * s;
        dst[RL_0] = q_ready[RL_0] - 0.05f * s;
    }

    void beginPhase(Phase next)
    {
        phase = next;
        phase_start_time = motiontime;
        captureCurrentTo(q_phase_from);
        std::cout << "[PHASE] -> " << phaseName(phase)
                  << " at t=" << motiontime * 2 << "ms" << std::endl;
    }

    void beginSafety(const std::string& reason)
    {
        if (phase == SAFETY_FREEZE || phase == SAFETY_TO_PRONE) return;
        safety_reason = reason;
        std::cout << "[SAFETY_FAULT] " << reason << std::endl;
        beginPhase(SAFETY_FREEZE);
        captureCurrentTo(q_freeze);
        setDesiredArray(q_freeze);
    }

    void handleAction(const std::string& action)
    {
        if (action.empty()) return;

        std::cout << "[ACTION] " << action << std::endl;

        if (action == "ready")
        {
            if (phase != READY_HOLD)
            {
                beginPhase(RAISE_TO_READY);
            }
            return;
        }

        if (action == "prone" || action == "choki" || action == "emergency_prone" || action == "stop")
        {
            beginPhase(LOWER_TO_PRONE);
            return;
        }

        if (action == "wave")
        {
            if (phase == READY_HOLD)
            {
                beginPhase(WAVE_UP);
            }
            else
            {
                std::cout << "[ACTION_IGNORED] wave requires READY_HOLD" << std::endl;
            }
            return;
        }

        if (action == "shake")
        {
            if (phase == READY_HOLD)
            {
                beginPhase(SHAKE_UP);
            }
            else
            {
                std::cout << "[ACTION_IGNORED] shake requires READY_HOLD" << std::endl;
            }
            return;
        }

        if (action == "sway")
        {
            if (phase == READY_HOLD)
            {
                beginPhase(SWAY_OSC);
            }
            else
            {
                std::cout << "[ACTION_IGNORED] sway requires READY_HOLD" << std::endl;
            }
            return;
        }

        std::cout << "[ACTION_UNKNOWN] " << action << std::endl;
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
            case PRONE_HOLD: return "PRONE_HOLD";
            case WAVE_UP: return "WAVE_UP";
            case WAVE_OSC: return "WAVE_OSC";
            case WAVE_RETURN: return "WAVE_RETURN";
            case SHAKE_UP: return "SHAKE_UP";
            case SHAKE_OSC: return "SHAKE_OSC";
            case SHAKE_RETURN: return "SHAKE_RETURN";
            case SWAY_OSC: return "SWAY_OSC";
            case SWAY_RETURN: return "SWAY_RETURN";
            case SAFETY_FREEZE: return "SAFETY_FREEZE";
            case SAFETY_TO_PRONE: return "SAFETY_TO_PRONE";
            default: return "UNKNOWN";
        }
    }

    void RobotControl()
    {
        motiontime++;

        udp.GetRecv(state);
        updateCommunicationMonitor();
        udp.InitCmdData(cmd);

        if (phase == CAPTURE)
        {
            if (stateValidBasic())
            {
                captureCurrentTo(q_prone);
                setDesiredArray(q_prone);
                makeReadyTarget();
                makeWaveTargets();
                makeShakeTargets();

                std::cout << "[CAPTURE] prone q captured" << std::endl;
                printStatus(true);
                beginPhase(PRONE_HOLD_START);
            }

            for (int i = 0; i < 12; i++)
            {
                setMotor(i, state.motorState[i].q, 0.0f, 1.0f);
            }
            udp.SetSend(cmd);
            return;
        }

        std::string fault;
        if (safetyFault(fault))
        {
            beginSafety(fault);
        }

        // Read external action only in non-safety phases.
        if (phase != SAFETY_FREEZE && phase != SAFETY_TO_PRONE)
        {
            std::string action = readAction();
            handleAction(action);
        }

        int phase_t = motiontime - phase_start_time;
        float kp = KP_HOLD;
        float kd = KD_HOLD;

        if (phase == PRONE_HOLD_START)
        {
            float r = smooth01(phase_t / 1000.0f); // 2 s ramp
            kp = KP_HOLD * r;
            setDesiredArray(q_prone);
            if (phase_t > 1000) beginPhase(RAISE_TO_READY);
        }
        else if (phase == RAISE_TO_READY)
        {
            interpolateArray(q_phase_from, q_ready, phase_t / 2500.0f); // 5 s
            if (phase_t > 2500) beginPhase(READY_HOLD);
        }
        else if (phase == READY_HOLD)
        {
            setDesiredArray(q_ready);
        }
        else if (phase == LOWER_TO_PRONE)
        {
            interpolateArray(q_phase_from, q_prone, phase_t / 1500.0f); // 3 s
            if (phase_t > 1500) beginPhase(PRONE_HOLD);
        }
        else if (phase == PRONE_HOLD)
        {
            setDesiredArray(q_prone);
        }
        else if (phase == WAVE_UP)
        {
            kp = KP_ACTION;
            kd = KD_ACTION;
            interpolateArray(q_phase_from, q_wave_up, phase_t / 600.0f); // 1.2 s
            if (phase_t > 600) beginPhase(WAVE_OSC);
        }
        else if (phase == WAVE_OSC)
        {
            kp = KP_ACTION;
            kd = KD_ACTION;
            float s = 0.5f + 0.5f * std::sin(2.0f * PI * phase_t / 450.0f);
            interpolateArray(q_wave_down, q_wave_up, s);
            if (phase_t > 1600) beginPhase(WAVE_RETURN); // 3.2 s
        }
        else if (phase == WAVE_RETURN)
        {
            kp = KP_ACTION;
            kd = KD_ACTION;
            interpolateArray(q_phase_from, q_ready, phase_t / 600.0f);
            if (phase_t > 600) beginPhase(READY_HOLD);
        }
        else if (phase == SHAKE_UP)
        {
            kp = KP_ACTION;
            kd = KD_ACTION;
            interpolateArray(q_phase_from, q_shake_up, phase_t / 600.0f);
            if (phase_t > 600) beginPhase(SHAKE_OSC);
        }
        else if (phase == SHAKE_OSC)
        {
            kp = KP_ACTION;
            kd = KD_ACTION;
            float s = 0.5f + 0.5f * std::sin(2.0f * PI * phase_t / 350.0f);
            interpolateArray(q_shake_down, q_shake_up, s);
            if (phase_t > 1400) beginPhase(SHAKE_RETURN);
        }
        else if (phase == SHAKE_RETURN)
        {
            kp = KP_ACTION;
            kd = KD_ACTION;
            interpolateArray(q_phase_from, q_ready, phase_t / 600.0f);
            if (phase_t > 600) beginPhase(READY_HOLD);
        }
        else if (phase == SWAY_OSC)
        {
            float s = std::sin(2.0f * PI * phase_t / 700.0f);
            makeSwayTarget(q_sway, s);
            setDesiredArray(q_sway);
            if (phase_t > 1400) beginPhase(SWAY_RETURN);
        }
        else if (phase == SWAY_RETURN)
        {
            interpolateArray(q_phase_from, q_ready, phase_t / 500.0f);
            if (phase_t > 500) beginPhase(READY_HOLD);
        }
        else if (phase == SAFETY_FREEZE)
        {
            kp = KP_SAFETY;
            kd = KD_SAFETY;
            setDesiredArray(q_freeze);
            if (phase_t > 250) beginPhase(SAFETY_TO_PRONE); // 0.5 s freeze
        }
        else if (phase == SAFETY_TO_PRONE)
        {
            kp = KP_SAFETY;
            kd = KD_SAFETY;
            interpolateArray(q_phase_from, q_prone, phase_t / 2000.0f); // 4 s
            if (phase_t > 2000) beginPhase(PRONE_HOLD);
        }

        for (int i = 0; i < 12; i++)
        {
            setMotor(i, q_des[i], kp, kd);
        }

        // Keep SDK safety. Do not use PositionProtect here: prone calf angles near
        // the A1 lower calf limit can trip it even in the intended prone posture.
        safe.PositionLimit(cmd);
        safe.PowerProtect(cmd, state, 1);

        udp.SetSend(cmd);

        if (motiontime % 250 == 0)
        {
            printStatus(false);
        }
    }

    void printStatus(bool all_motors)
    {
        std::cout << "t=" << motiontime * 2 << "ms"
                  << " phase=" << phaseName(phase)
                  << " roll=" << state.imu.rpy[0] * RAD2DEG
                  << " pitch=" << state.imu.rpy[1] * RAD2DEG
                  << " gyroMax=" << maxAbsGyro()
                  << " dqMax=" << maxAbsDq()
                  << " qErrMax=" << maxAbsQError()
                  << " foot=[" << state.footForce[0] << ","
                                << state.footForce[1] << ","
                                << state.footForce[2] << ","
                                << state.footForce[3] << "]";
        if (!safety_reason.empty()) std::cout << " safety_reason=" << safety_reason;
        std::cout << std::endl;

        if (all_motors)
        {
            for (int i = 0; i < 12; i++)
            {
                std::cout << "  m" << std::setw(2) << i
                          << " q=" << std::setw(10) << state.motorState[i].q
                          << " dq=" << std::setw(10) << state.motorState[i].dq
                          << " tau=" << std::setw(10) << state.motorState[i].tauEst
                          << std::endl;
            }
        }
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
    float q_phase_from[12] = {0.0f};
    float q_freeze[12] = {0.0f};
    float q_wave_up[12] = {0.0f};
    float q_wave_down[12] = {0.0f};
    float q_shake_up[12] = {0.0f};
    float q_shake_down[12] = {0.0f};
    float q_sway[12] = {0.0f};

    std::string safety_reason;

    uint32_t last_tick = 0;
    int same_tick_count = 0;
    bool tick_initialized = false;

    const char* ACTION_PATH = "/tmp/a1_action";
    const char* CHOKI_TRIGGER_PATH = "/tmp/a1_choki_trigger";

    const float HIP_OFFSET_READY = 0.18f;
    const float KNEE_OFFSET_READY = 0.90f;

    const float KP_HOLD = 55.0f;
    const float KD_HOLD = 2.5f;
    const float KP_ACTION = 45.0f;
    const float KD_ACTION = 3.0f;
    const float KP_SAFETY = 35.0f;
    const float KD_SAFETY = 4.0f;

    const float MAX_ROLL_DEG = 18.0f;
    const float MAX_PITCH_DEG = 18.0f;
    const float MAX_GYRO_RAD_S = 2.5f;
    const float MAX_DQ_RAD_S = 14.0f;
    const float MAX_Q_ERROR_RAD = 0.85f;
    const float FOOT_FORCE_CONTACT_TH = 5.0f;
    const int MAX_SAME_TICK_COUNT = 100;
    const float RAD2DEG = 57.2957795f;
    const float PI = 3.14159265358979323846f;

    float dt = 0.002f;
};

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "A1 ACTION SAFETY FSM." << std::endl
              << "Start from prone / low posture." << std::endl
              << "Actions: ready, prone, choki, emergency_prone, stop, wave, shake, sway" << std::endl
              << "Action path: /tmp/a1_action" << std::endl
              << "Legacy trigger: /tmp/a1_choki_trigger" << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cin.ignore();

    std::remove("/tmp/a1_action");
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
