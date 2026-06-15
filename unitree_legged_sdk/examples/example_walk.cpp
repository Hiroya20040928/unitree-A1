/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    int last_report_ms = -1000;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

    cmd.forwardSpeed = 0.0f;
    cmd.sideSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.bodyHeight = 0.0f;

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.roll  = 0;
    cmd.pitch = 0;
    cmd.yaw = 0;

    if(motiontime < 1000){
        cmd.mode = 1;
    }

    if(motiontime >= 1000 && motiontime < 2000){
        cmd.mode = 2;
    }

    if(motiontime >= 2000 && motiontime < 7000){
        cmd.mode = 2;
        cmd.forwardSpeed = 0.2f;
    }

    if(motiontime >= 7000 && motiontime < 9000){
        cmd.mode = 2;
        cmd.rotateSpeed = 0.25f;
    }

    if(motiontime >= 9000){
        cmd.mode = 1;
    }

    if(motiontime - last_report_ms >= 1000){
        last_report_ms = motiontime;
        std::cout << "t=" << motiontime
                  << "ms send(mode=" << static_cast<int>(cmd.mode)
                  << ", fwd=" << cmd.forwardSpeed
                  << ", rot=" << cmd.rotateSpeed
                  << ") recv(mode=" << static_cast<int>(state.mode)
                  << ", fwd=" << state.forwardSpeed
                  << ", tick=" << state.tick
                  << ")" << std::endl;
    }

    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    // Avoid pinning to a non-existent CPU on small systems.
    LoopFunc loop_udpSend("udp_send",     custom.dt,    boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt,    boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
