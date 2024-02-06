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
        rospy.Subscriber("/human_goal", PoseStamped, self.pose_callback)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        rospy.loginfo("human_follow node initialized")
        
    def pose_callback(self, msg):
        self.goalx = msg.point.x
        self.goaly = msg.point.y
        self.goalz = msg.point.z
        self.goalo = msg.orientation
        
        goal_point.pose.position.x = self.goalx
        goal_point.pose.position.y = self.goaly
        goal_point.pose.position.z = self.goalz
                
        quaternion = self.goalo
                
        goal_point.pose.orientation.x = quaternion[0]
        goal_point.pose.orientation.y = quaternion[1]
        goal_point.pose.orientation.z = quaternion[2]
        goal_point.pose.orientation.w = quaternion[3]
        
        rospy.loginfo("Goal Point: x=%f, y=%f, z=%f" % (goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z))
                
        try:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = goal_point.pose.position
            goal.target_pose.pose.orientation = goal_point.pose.orientation
        
            self.client.send_goal(goal)
            rospy.loginfo("Goal point sent to move_base")
            self.client.wait_for_result()
            
        except:
            rospy.logerr("Error sending goal to move_base")

def main():
    rospy.sleep(2.0)  # Wait for tf listener to initialize
    HumanFollow()
    rospy.spin()

if __name__ == '__main__':
    main()