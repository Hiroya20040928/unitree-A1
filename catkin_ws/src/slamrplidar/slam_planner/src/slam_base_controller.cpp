/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>
#include <unistd.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

class BaseControllerNode
{
public:
    BaseControllerNode()
      : nh_()
      , private_nh_("~")
      , udp_(HIGHLEVEL)
    {
        target_ip_ = UDP_SERVER_IP_BASIC;
        target_port_ = UDP_SERVER_PORT;
        local_port_ = UDP_CLIENT_PORT;
        udp_.InitCmdData(cmd_);

        private_nh_.param("control_rate_hz", control_rate_hz_, 500.0);
        private_nh_.param("command_timeout", command_timeout_, 0.25);
        private_nh_.param("max_forward_speed", max_forward_speed_, 0.25);
        private_nh_.param("max_rotate_speed", max_rotate_speed_, 0.60);
        private_nh_.param("body_height", body_height_, 0.0);
        private_nh_.param("invert_yaw_sign", invert_yaw_sign_, true);
        private_nh_.param("startup_stand_time", startup_stand_time_, 1.0);

        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &BaseControllerNode::cmdCallback, this);
        recv_thread_ = std::thread(&BaseControllerNode::recvLoop, this);
        recv_thread_.detach();

        last_cmd_time_ = ros::Time(0);
        last_report_time_ = ros::Time(0);
        start_time_ = ros::Time::now();
        last_state_tick_ = 0;

        ROS_INFO("base_controller_node configured:"
                 " target_ip=%s target_port=%d local_port=%d control_rate=%.1fHz timeout=%.2fs",
                 target_ip_.c_str(), target_port_, local_port_, control_rate_hz_, command_timeout_);
    }

    void spin()
    {
        ros::Rate rate(control_rate_hz_);
        while (ros::ok()) {
            udp_.GetRecv(state_);

            ros::spinOnce();
            updateCommandFromTwist();
            udp_.SetSend(cmd_);
            udp_.Send();
            maybeReport();

            rate.sleep();
        }

        stopRobot();
    }

private:
    void recvLoop()
    {
        while (ros::ok()) {
            udp_.Recv();
            usleep(2000);
        }
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_cmd_ = *msg;
        last_cmd_time_ = ros::Time::now();
    }

    void updateCommandFromTwist()
    {
        const ros::Time now = ros::Time::now();

        cmd_.forwardSpeed = 0.0f;
        cmd_.sideSpeed = 0.0f;
        cmd_.rotateSpeed = 0.0f;
        cmd_.bodyHeight = body_height_;
        cmd_.mode = 1;  // forced stand by default
        cmd_.roll = 0.0f;
        cmd_.pitch = 0.0f;
        cmd_.yaw = 0.0f;

        if ((now - start_time_).toSec() < startup_stand_time_) {
            return;
        }

        if (!commandActive(now)) {
            return;
        }

        const double forward = clamp(last_cmd_.linear.x, -max_forward_speed_, max_forward_speed_);
        double rotate = clamp(last_cmd_.angular.z, -max_rotate_speed_, max_rotate_speed_);
        if (invert_yaw_sign_) {
            rotate = -rotate;
        }

        if (std::fabs(forward) < 1e-3 && std::fabs(rotate) < 1e-3) {
            return;
        }

        cmd_.mode = 2;  // walk continuously
        cmd_.forwardSpeed = static_cast<float>(forward);
        cmd_.rotateSpeed = static_cast<float>(rotate);
    }

    bool commandActive(const ros::Time& now) const
    {
        if (last_cmd_time_.isZero()) {
            return false;
        }
        return (now - last_cmd_time_).toSec() <= command_timeout_;
    }

    void maybeReport()
    {
        const ros::Time now = ros::Time::now();
        if (!last_report_time_.isZero() && (now - last_report_time_).toSec() < 1.0) {
            return;
        }
        last_report_time_ = now;

        if (state_.tick == last_state_tick_) {
            ROS_WARN("base_controller_node state tick is not changing (tick=%u). Check Unitree UDP link to %s:%d.",
                     state_.tick, target_ip_.c_str(), target_port_);
        } else {
            ROS_INFO("base_controller_node send(mode=%u fwd=%.3f rot=%.3f) recv(mode=%u fwd=%.3f tick=%u)",
                     static_cast<unsigned int>(cmd_.mode),
                     cmd_.forwardSpeed,
                     cmd_.rotateSpeed,
                     static_cast<unsigned int>(state_.mode),
                     state_.forwardSpeed,
                     state_.tick);
        }

        last_state_tick_ = state_.tick;
    }

    void stopRobot()
    {
        cmd_.mode = 1;
        cmd_.forwardSpeed = 0.0f;
        cmd_.sideSpeed = 0.0f;
        cmd_.rotateSpeed = 0.0f;
        cmd_.bodyHeight = body_height_;
        cmd_.roll = 0.0f;
        cmd_.pitch = 0.0f;
        cmd_.yaw = 0.0f;

        for (int i = 0; i < 20; ++i) {
            udp_.SetSend(cmd_);
            udp_.Send();
            usleep(2000);
        }
    }

    static double clamp(double value, double lower, double upper)
    {
        return std::max(lower, std::min(value, upper));
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cmd_sub_;
    std::thread recv_thread_;

    UDP udp_;
    HighCmd cmd_ = {0};
    HighState state_ = {0};
    geometry_msgs::Twist last_cmd_;

    ros::Time start_time_;
    ros::Time last_cmd_time_;
    ros::Time last_report_time_;

    std::string target_ip_;
    int local_port_ = 8081;
    int target_port_ = UDP_SERVER_PORT;
    double control_rate_hz_ = 500.0;
    double command_timeout_ = 0.25;
    double max_forward_speed_ = 0.25;
    double max_rotate_speed_ = 0.60;
    double body_height_ = 0.0;
    double startup_stand_time_ = 1.0;
    bool invert_yaw_sign_ = true;
    uint32_t last_state_tick_ = 0;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "base_controller_node");
    BaseControllerNode node;
    node.spin();
    return 0;
}
