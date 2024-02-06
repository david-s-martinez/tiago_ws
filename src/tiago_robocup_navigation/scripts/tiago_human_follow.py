#!/usr/bin/env python3
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import sys
import numpy as np
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class HumanFollow:
    def __init__(self):
        # Initialize MoveIt! Commander and ROS nodes
        
        rospy.init_node('human_follow', anonymous=True)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
 
        
        rospy.Subscriber("/human_goal", PoseStamped, self.pose_callback)
        
        
        rospy.loginfo("human_follow node initialized")
        
    def pose_callback(self, msg):
        self.goal_pose = msg.pose.position
        self.goal_ori = msg.pose.orientation

                
        try:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = self.goal_pose
            goal.target_pose.pose.orientation = self.goal_ori
        
            self.client.send_goal(goal)
            rospy.loginfo("Goal point sent to move_base")
            self.client.wait_for_result()
        except:
            rospy.loginfo("Error in sending goal point to move_base")

def main():
    rospy.sleep(2.0)  # Wait for tf listener to initialize
    HumanFollow()
    rospy.spin()

if __name__ == '__main__':
    main()