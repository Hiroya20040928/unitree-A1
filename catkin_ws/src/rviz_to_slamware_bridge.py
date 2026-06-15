#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from slamware_ros_sdk.msg import MoveToRequest

def callback(msg):
    rospy.loginfo("Rvizから矢印を検知！ 座標: X=%.3f, Y=%.3f", msg.pose.position.x, msg.pose.position.y)
    
    # Slamwareの独自型にデータを詰め替える
    req = MoveToRequest()
    req.location.x = msg.pose.position.x
    req.location.y = msg.pose.position.y
    req.location.z = 0.0
    req.options.opt_flags.flags = 0
    req.options.speed_ratio.is_valid = False
    req.options.speed_ratio.value = 0.0
    req.yaw = 0.0
    
    # Slamwareへ直撃パブリッシュ
    pub.publish(req)
    rospy.loginfo("Slamware独自型へ変換して送信完了。")

if __name__ == '__main__':
    rospy.init_node('rviz_to_slamware_bridge_node')
    
    # 送信口（Slamware独自型）
    pub = rospy.Publisher('/slamware_ros_sdk_server_node/move_to', MoveToRequest, queue_size=10)
    
    # 受取口（Rviz標準型）
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
    
    rospy.loginfo("=== Rviz型変換ブリッジ起動完了：矢印待機中 ===")
    rospy.spin()
