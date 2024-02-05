#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_arm(arm_positions):
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
                                  'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    arm_point = JointTrajectoryPoint()
    arm_point.positions = arm_positions
    arm_point.time_from_start = rospy.Duration(1.0)
    arm_trajectory.points.append(arm_point)

    arm_pub.publish(arm_trajectory)
    rospy.loginfo("Arm command sent.")

def move_head(head_positions):
    head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    head_trajectory = JointTrajectory()
    head_trajectory.joint_names = ['head_1_joint', 'head_2_joint']

    head_point = JointTrajectoryPoint()
    head_point.positions = head_positions
    head_point.time_from_start = rospy.Duration(1.0)
    head_trajectory.points.append(head_point)

    head_pub.publish(head_trajectory)
    rospy.loginfo("Head command sent.")

def move_gripper(gripper_positions):
    gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    gripper_trajectory = JointTrajectory()
    gripper_trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = gripper_positions
    gripper_point.time_from_start = rospy.Duration(1.0)
    gripper_trajectory.points.append(gripper_point)

    gripper_pub.publish(gripper_trajectory)
    rospy.loginfo("Gripper command sent.")

def move_torso(torso_position):
    torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    torso_trajectory = JointTrajectory()
    torso_trajectory.joint_names = ['torso_lift_joint']

    torso_point = JointTrajectoryPoint()
    torso_point.positions = [torso_position]
    torso_point.time_from_start = rospy.Duration(1.0)
    torso_trajectory.points.append(torso_point)

    torso_pub.publish(torso_trajectory)
    rospy.loginfo("Torso move command sent to position: %s", torso_point.positions)

if __name__ == '__main__':
    try:
        rospy.init_node('tiago_arm_and_gripper_mover', anonymous=True)

        # Example positions - Replace these with the desired positions
        arm_positions = [0.4, -1.17, -1.9, 2.3, -1.3, -0.45, 1.75]
        head_positions = [0.7, -0.8]
        gripper_positions = [0, 0.0]
        torso_position = 0.2

        move_head(head_positions)
        move_arm(arm_positions)
        move_gripper(gripper_positions)
        move_torso(torso_position)
    except rospy.ROSInterruptException:
        pass

