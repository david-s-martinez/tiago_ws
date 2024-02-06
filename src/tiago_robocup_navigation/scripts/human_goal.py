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

class GetTargetPoint:
    def __init__(self):
        # Initialize MoveIt! Commander and ROS nodes
 
        rospy.init_node('get_target_point', anonymous=True)
        rospy.Subscriber("/goal_point", PointStamped, self.point_callback)
        rospy.Subscriber("/robot_pose", PointStamped, self.pose_callback)

        # Create tf listener
        self.listener = tf.TransformListener()
        self.pick_x = None
        self.pick_y = None
        self.pick_z = None
        
        rospy.loginfo("GetTargetPoint node initialized")

    def point_callback(self, msg):
        self.pick_x = msg.point.x
        self.pick_y = msg.point.y
        self.pick_z = msg.point.z
        self.goal_callback()
        
    def pose_callback(self, msg):
        self.robot_x = msg.point.x
        self.robot_y = msg.point.y
        self.robot_z = msg.point.z
        self.robot_o = msg.orientation
        rospy.loginfo("Robot Position: x=%f, y=%f, z=%f" % (self.robot_x, self.robot_y, self.robot_z))
        self.goal_callback()
    
    def goal_callback(self):
        try:
            self.getHumanGoal()
        except:
            rospy.logerr("Error in transform_and_move")
               
    def getHumanGoal(self):
        
        camera_point = geometry_msgs.msg.PointStamped()
        camera_point.header.frame_id = "xtion_rgb_optical_frame"
        # camera_point.header.stamp = rospy.Time.now()
        camera_point.point.x = self.pick_x
        camera_point.point.y = self.pick_y
        camera_point.point.z = self.pick_z
        if (camera_point.point.x == None or camera_point.point.y == None or camera_point.point.z == None):
            # rospy.loginfo("No target point detected")
            return
        # rospy.loginfo("Transformed Point: x=%f, y=%f, z=%f" % (camera_point.point.x, camera_point.point.y, camera_point.point.z))
        try:
            # Wait until the transform is available
            self.listener.waitForTransform("map", camera_point.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            
            # Perform transformation
            transformed_point = self.listener.transformPoint("map", camera_point)
            
            # Publish the transformed point
            goal_point = geometry_msgs.msg.PoseStamped()
            goal_point.header.frame_id = "map"
            goal_point.header.stamp = rospy.Time.now()
            
            if transformed_point is not None:
                # rospy.loginfo("Transformed Point: x=%f, y=%f, z=%f" % (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))
                
                goal_point.pose.position.x = transformed_point.point.x
                goal_point.pose.position.y = transformed_point.point.y
                goal_point.pose.position.y = goal_point.pose.position.y - 0.5 * math.sin(math.atan2(transformed_point.point.y, transformed_point.point.x))
                goal_point.pose.position.z = transformed_point.point.z
                
                quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan2(transformed_point.point.y, transformed_point.point.x))
                goal_point.pose.orientation.x = quaternion[0]
                goal_point.pose.orientation.y = quaternion[1]
                goal_point.pose.orientation.z = quaternion[2]
                goal_point.pose.orientation.w = quaternion[3]
                rospy.loginfo("Goal point: x=%f, y=%f, z=%f" % (goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z))
                
                # Make the goal point 30 cm in front of the detected point
                # goal_point.pose.position.x = goal_point.pose.position.x + 0.5 * math.cos(math.atan2(transformed_point.point.y, transformed_point.point.x))
                
                
                
            else:
                # subscribe to robot's current position
                rospy.loginfo("Human Not Found")
                
            self.goal_pub = rospy.Publisher('/human_goal', PoseStamped, queue_size=4)
            self.goal_pub.publish(goal_point)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Transform Error: %s" % e)

def main():
    rospy.sleep(2.0)  # Wait for tf listener to initialize
    GetTargetPoint()
    rospy.spin()

if __name__ == '__main__':
    main()