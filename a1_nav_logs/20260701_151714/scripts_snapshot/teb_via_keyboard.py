#!/usr/bin/env python

from __future__ import print_function

import errno
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


MSG = """
teb_via_keyboard
----------------
w/s : forward/back
a/d : rotate left/right
x or space : stop
CTRL-C : quit
"""


def get_key(timeout):
    tty.setraw(sys.stdin.fileno())
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    except select.error as exc:
        if exc[0] == errno.EINTR:
            key = '\x03'
        else:
            raise
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
    return key


def publish_keyboard():
    rospy.init_node('teb_via_keyboard')
    topic = rospy.get_param('~cmd_vel_topic', '/teb_via_teleop/cmd_vel').strip()
    if not topic or topic == '/':
        rospy.logwarn("Invalid ~cmd_vel_topic '%s'; falling back to /teb_via_teleop/cmd_vel", topic)
        topic = '/teb_via_teleop/cmd_vel'
    pub = rospy.Publisher(topic, Twist, queue_size=1)

    linear_speed = rospy.get_param('~linear_speed', 0.20)
    angular_speed = rospy.get_param('~angular_speed', 0.60)
    hold_timeout = rospy.get_param('~hold_timeout', 0.20)
    rate_hz = rospy.get_param('~rate', 15.0)

    print(MSG)
    print('publishing to {}'.format(topic))

    current = Twist()
    last_key_time = rospy.Time(0)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        key = get_key(0.05)

        if key == '\x03':
            break
        elif key == 'w':
            current.linear.x = linear_speed
            current.angular.z = 0.0
            last_key_time = rospy.Time.now()
        elif key == 's':
            current.linear.x = -linear_speed
            current.angular.z = 0.0
            last_key_time = rospy.Time.now()
        elif key == 'a':
            current.linear.x = 0.0
            current.angular.z = angular_speed
            last_key_time = rospy.Time.now()
        elif key == 'd':
            current.linear.x = 0.0
            current.angular.z = -angular_speed
            last_key_time = rospy.Time.now()
        elif key == 'x' or key == ' ':
            current = Twist()
            last_key_time = rospy.Time(0)

        if not last_key_time.is_zero():
            if (rospy.Time.now() - last_key_time).to_sec() > hold_timeout:
                current = Twist()

        pub.publish(current)
        rate.sleep()

    pub.publish(Twist())


if __name__ == '__main__':
    SETTINGS = termios.tcgetattr(sys.stdin)
    try:
        publish_keyboard()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
