#!/usr/bin/env python3
import rospy
import moveit_commander
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from actionlib_msgs.msg import GoalStatusArray
import sys
import numpy as np

class ArmController:
    def __init__(self):
        # Initialize MoveIt! Commander and ROS nodes
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.sleep(10)
        #rospy.init_node('tiago_arm_controller', anonymous=True)
        rospy.Subscriber("/pick_point", PointStamped, self.point_callback)
        rospy.Subscriber('/move_group/status', GoalStatusArray, self.status_callback)

        # Initialize the MoveGroupCommander object
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")
        rospy.sleep(10)
        # Set reference coordinate system
        self.arm_group.set_pose_reference_frame("base_footprint")

        # Create tf listener
        self.listener = tf.TransformListener()

        self.pick_x = None
        self.pick_y = None
        self.pick_z = None

        self.move_status = False

    def status_callback(self, msg):
        status = msg.status_list
        for s in status:
            state = s.status 
            if state == 3:
                self.move_status = True
            else:
                self.move_status = False

    def point_callback(self, msg):
        self.pick_x = msg.point.x
        self.pick_y = msg.point.y
        self.pick_z = msg.point.z

    def move_arm(self, position, orientation):
        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]
        
        # Convert RPY to quaternion
        quaternion = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        # Set target poses and plan movements
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()

        self.move_status = None
        self.arm_group.go(wait=True)
        self.arm_group.clear_pose_targets()

        rospy.sleep(2)

        rospy.loginfo(self.move_status)

        if self.move_status:
            return True
        else:
            return False

    # def transform_and_move(self,camera_point):
    def transform_and_move(self):
        camera_point = geometry_msgs.msg.PointStamped()
        camera_point.header.frame_id = "xtion_rgb_optical_frame"
        # camera_point.header.stamp = rospy.Time.now()
        camera_point.point.x = self.pick_x
        camera_point.point.y = self.pick_y
        camera_point.point.z = self.pick_z
        # rospy.loginfo("Transformed Point: x=%f, y=%f, z=%f" % (camera_point.point.x, camera_point.point.y, camera_point.point.z))
        try:
            # Wait until the transform is available
            self.listener.waitForTransform("base_footprint", camera_point.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            
            # Perform transformation
            transformed_point = self.listener.transformPoint("base_footprint", camera_point)
            
            if transformed_point is not None:
                rospy.loginfo("Transformed Point: x=%f, y=%f, z=%f" % (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))

                # Define target position and attitude
                position = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z+0.43]  # X, Y, Z

                orientation = [0, 1.6, 0]  # R, P, Y 
                try:
                    self.move_arm(position, orientation)
                except:
                    rospy.loginfo("Failed to move arm")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Transform Error: %s" % e)

def main():
    arm_controller = ArmController()

    # Create a PointStamped message representing the point to be converted
    # camera_point = geometry_msgs.msg.PointStamped()
    # camera_point.header.frame_id = "xtion_rgb_optical_frame"
    # camera_point.header.stamp = rospy.Time.now()
    # x: -0.15910915807115716
    # y: 0.15334252915969293
    # z: 0.774
    # camera_point.point.x = -0.15910915807115716
    # camera_point.point.y = 0.15334252915969293
    # camera_point.point.z = 0.774

    rospy.sleep(2.0)  # Wait for tf listener to initialize

    arm_controller.transform_and_move()
    # arm_controller.transform_and_move(camera_point)


    rospy.spin()



if __name__ == '__main__':
    main()
