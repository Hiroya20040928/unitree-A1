#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class A1CmdVelBias:
    def __init__(self):
        self.v_ref = rospy.get_param("~v_ref", 1.0)
        self.k_bias = rospy.get_param("~k_bias", 0.14)
        self.min_vx_for_bias = rospy.get_param("~min_vx_for_bias", 0.05)

        self.max_abs_bias = rospy.get_param("~max_abs_bias", 0.14)
        self.max_abs_vx = rospy.get_param("~max_abs_vx", 0.95)
        self.max_abs_wz = rospy.get_param("~max_abs_wz", 0.35)

        self.watchdog_timeout = rospy.get_param("~watchdog_timeout", 0.35)

        self.last_msg_time = rospy.Time(0)
        self.last_out = Twist()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/cmd_vel_raw", Twist, self.cb, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.watchdog)

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def cb(self, msg):
        out = Twist()

        vx = self.clamp(msg.linear.x, -self.max_abs_vx, self.max_abs_vx)
        wz = self.clamp(msg.angular.z, -self.max_abs_wz, self.max_abs_wz)

        bias = 0.0

        # 前進時だけ補正する．停止・その場旋回・後退には補正を入れない．
        if vx > self.min_vx_for_bias and vx < self.v_ref:
            bias = -self.k_bias * (self.v_ref - vx)
            bias = self.clamp(bias, -self.max_abs_bias, self.max_abs_bias)

        out.linear.x = vx
        out.linear.y = 0.0
        out.linear.z = 0.0

        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = self.clamp(wz + bias, -self.max_abs_wz, self.max_abs_wz)

        self.last_msg_time = rospy.Time.now()
        self.last_out = out
        self.pub.publish(out)

    def watchdog(self, event):
        if self.last_msg_time == rospy.Time(0):
            return

        if (rospy.Time.now() - self.last_msg_time).to_sec() > self.watchdog_timeout:
            zero = Twist()
            self.pub.publish(zero)

if __name__ == "__main__":
    rospy.init_node("a1_cmd_vel_bias")
    A1CmdVelBias()
    rospy.spin()
