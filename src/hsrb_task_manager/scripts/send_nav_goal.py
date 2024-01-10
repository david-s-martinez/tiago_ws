#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf

def move_to_goal(x, y, z,w1,w2,w3):
    # 初始化 move_base action 客户端
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # 等待服务器可用
    client.wait_for_server()

    # 创建一个新的目标
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quaternion = tf.transformations.quaternion_from_euler(w1,w2,w3)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]


    # 发送目标到 move_base
    client.send_goal(goal)
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_goal_sender')
        result = move_to_goal(1.45, -2.71, 0.0,0.0,0.0,0.0)  # 修改为您的目标点
        if result:            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
