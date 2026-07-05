/************************************************************************
A1 High-level follow driver.
Reads /tmp/a1_follow_cmd written by HTTP camera server:
  enable vx vy wz timestamp
and sends Unitree HighCmd in HIGHLEVEL mode.

Designed as a safer alternative to low-level joint control for walking.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

static double nowSec()
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

static float clampf(float x, float lo, float hi)
{
    return std::max(lo, std::min(hi, x));
}

static float slew(float cur, float target, float max_step)
{
    float d = target - cur;
    if (d > max_step) return cur + max_step;
    if (d < -max_step) return cur - max_step;
    return target;
}

struct FollowCmd
{
    bool enable = false;
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    double ts = -1.0;
    bool valid = false;
};

class A1HighFollow
{
public:
    A1HighFollow(uint8_t level)
        : safe(LeggedType::A1), udp(level)
    {
        udp.InitCmdData(cmd);
    }

    void UDPRecv() { udp.Recv(); }
    void UDPSend() { udp.Send(); }
    void RobotControl();

private:
    FollowCmd readFollowCmd();
    float readFrontObstacleM();
    bool safetyOK(std::string &reason);
    void setHighCmd(int mode, float vx, float vy, float wz);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};

    int motiontime_ms = 0;
    int last_report_ms = -1000;
    double start_sec = nowSec();

    float cur_vx = 0.0f;
    float cur_vy = 0.0f;
    float cur_wz = 0.0f;

    const float dt = 0.002f;

    const std::string follow_path = "/tmp/a1_follow_cmd";
    const std::string obstacle_path = "/tmp/a1_obstacle_front_m";
};

FollowCmd A1HighFollow::readFollowCmd()
{
    FollowCmd fc;
    std::ifstream ifs(follow_path.c_str());
    if (!ifs.good()) return fc;

    int en = 0;
    ifs >> en >> fc.vx >> fc.vy >> fc.wz >> fc.ts;
    if (!ifs.good()) return fc;

    fc.enable = (en != 0);
    fc.valid = true;

    // Conservative clamps. HighCmd speed fields are normalized scales (-1..1),
    // not guaranteed SI m/s values on all firmware versions.
    fc.vx = clampf(fc.vx, 0.0f, 0.22f);     // no backward motion in first version
    fc.vy = clampf(fc.vy, -0.08f, 0.08f);   // disabled by vision side, kept tiny
    fc.wz = clampf(fc.wz, -0.35f, 0.35f);

    double age = nowSec() - fc.ts;
    if (age > 0.45) {
        fc.enable = false;
        fc.vx = fc.vy = fc.wz = 0.0f;
    }

    return fc;
}

float A1HighFollow::readFrontObstacleM()
{
    std::ifstream ifs(obstacle_path.c_str());
    if (!ifs.good()) return 999.0f;
    float d = 999.0f;
    ifs >> d;
    if (!ifs.good()) return 999.0f;
    return d;
}

bool A1HighFollow::safetyOK(std::string &reason)
{
    float roll = state.imu.rpy[0];
    float pitch = state.imu.rpy[1];
    float gyroMax = 0.0f;
    for (int i = 0; i < 3; ++i) gyroMax = std::max(gyroMax, std::fabs(state.imu.gyroscope[i]));

    if (std::fabs(roll) > 0.35f) {
        reason = "roll_over";
        return false;
    }
    if (std::fabs(pitch) > 0.35f) {
        reason = "pitch_over";
        return false;
    }
    if (gyroMax > 2.8f) {
        reason = "gyro_over";
        return false;
    }

    float front = readFrontObstacleM();
    if (front < 0.70f) {
        reason = "front_obstacle";
        return false;
    }

    reason = "ok";
    return true;
}

void A1HighFollow::setHighCmd(int mode, float vx, float vy, float wz)
{
    cmd.mode = mode;
    cmd.forwardSpeed = vx;
    cmd.sideSpeed = vy;
    cmd.rotateSpeed = wz;
    cmd.bodyHeight = 0.0f;
    cmd.footRaiseHeight = 0.0f;
    cmd.roll = 0.0f;
    cmd.pitch = 0.0f;
    cmd.yaw = 0.0f;
}

void A1HighFollow::RobotControl()
{
    motiontime_ms += 2;
    udp.GetRecv(state);

    FollowCmd fc = readFollowCmd();
    std::string safety_reason;
    bool ok = safetyOK(safety_reason);

    // First force stand briefly after launch.
    bool warmup = (nowSec() - start_sec) < 1.5;

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;
    int target_mode = 1; // forced stand / idle standing

    if (!warmup && ok && fc.valid && fc.enable) {
        target_mode = 2; // walk continuously
        target_vx = fc.vx;
        target_vy = fc.vy;
        target_wz = fc.wz;
    }

    // Smooth commands. Prevent sudden step commands from vision jitter.
    cur_vx = slew(cur_vx, target_vx, 0.0025f);
    cur_vy = slew(cur_vy, target_vy, 0.0015f);
    cur_wz = slew(cur_wz, target_wz, 0.0040f);

    if (target_mode != 2) {
        // When stopped, decay to exactly zero.
        if (std::fabs(cur_vx) < 0.003f) cur_vx = 0.0f;
        if (std::fabs(cur_vy) < 0.003f) cur_vy = 0.0f;
        if (std::fabs(cur_wz) < 0.003f) cur_wz = 0.0f;
    }

    setHighCmd(target_mode, cur_vx, cur_vy, cur_wz);

    if (motiontime_ms - last_report_ms >= 500) {
        last_report_ms = motiontime_ms;
        std::cout << "t=" << motiontime_ms
                  << " mode=" << static_cast<int>(cmd.mode)
                  << " vx=" << cmd.forwardSpeed
                  << " vy=" << cmd.sideSpeed
                  << " wz=" << cmd.rotateSpeed
                  << " follow=" << (fc.enable ? 1 : 0)
                  << " valid=" << (fc.valid ? 1 : 0)
                  << " safety=" << safety_reason
                  << " roll=" << state.imu.rpy[0]
                  << " pitch=" << state.imu.rpy[1]
                  << " tick=" << state.tick
                  << std::endl;
    }

    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "A1 High-level person-follow driver" << std::endl;
    std::cout << "WARNING: This sends HIGHLEVEL walking velocity commands." << std::endl;
    std::cout << "Use a wide open area. Keep your hand on emergency stop / power." << std::endl;
    std::cout << "Press Enter to start..." << std::endl;
    std::cin.ignore();

    A1HighFollow robot(HIGHLEVEL);
    InitEnvironment();

    LoopFunc loop_control("control_loop", 0.002, boost::bind(&A1HighFollow::RobotControl, &robot));
    LoopFunc loop_udpSend("udp_send",     0.002, boost::bind(&A1HighFollow::UDPSend, &robot));
    LoopFunc loop_udpRecv("udp_recv",     0.002, boost::bind(&A1HighFollow::UDPRecv, &robot));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (true) sleep(10);
    return 0;
}
