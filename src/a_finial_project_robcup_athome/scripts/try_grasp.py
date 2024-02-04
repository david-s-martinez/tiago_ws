#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_arm():
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
                                  'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    arm_point = JointTrajectoryPoint()
    arm_point.positions = [0.4, -1.17, -1.9, 2.3, -1.3, -0.45, 1.75]
    arm_point.time_from_start = rospy.Duration(1.0)
    arm_trajectory.points.append(arm_point)

    arm_pub.publish(arm_trajectory)
    rospy.loginfo("Arm command sent.")

def move_gripper():
    gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    gripper_trajectory = JointTrajectory()
    gripper_trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [0, 0.0]
    gripper_point.time_from_start = rospy.Duration(1.0)
    gripper_trajectory.points.append(gripper_point)

    gripper_pub.publish(gripper_trajectory)
    rospy.loginfo("Gripper command sent.")

def move_torso():
    torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    torso_trajectory = JointTrajectory()
    torso_trajectory.joint_names = ['torso_lift_joint']

    torso_point = JointTrajectoryPoint()
    torso_point.positions = [0.2]
    torso_point.time_from_start = rospy.Duration(1.0)
    torso_trajectory.points.append(torso_point)

    torso_pub.publish(torso_trajectory)
    rospy.loginfo("Torso move command sent to position: %s",  torso_point.positions)

if __name__ == '__main__':
    try:
        rospy.init_node('tiago_arm_and_gripper_mover', anonymous=True)
        move_arm()
        move_gripper()
        move_torso()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python3

# import sys
# import rospy
# import moveit_commander
# import geometry_msgs.msg
# import moveit_msgs.msg

# class TiagoArmMover:
#     def __init__(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('tiago_arm_mover', anonymous=True)

#         robot = moveit_commander.RobotCommander()
#         scene = moveit_commander.PlanningSceneInterface()
#         group_name = "arm_torso"  # or whatever your group is
#         move_group = moveit_commander.MoveGroupCommander(group_name)

#         self.move_group = move_group

#     def move_arm_up(self, distance):
#         # Get the current pose so we can add the distance to the z-component
#         pose_goal = self.move_group.get_current_pose().pose

#         # Increase the z-component by the distance we want to move up
#         pose_goal.position.z += distance

#         # Set the new pose as the target
#         self.move_group.set_pose_target(pose_goal)

#         # Plan and execute the motion
#         plan = self.move_group.go(wait=True)
#         self.move_group.stop()  # Ensure no residual movement
#         self.move_group.clear_pose_targets()  # Clear targets after movement

#         return plan

# def main():
#     try:
#         arm_mover = TiagoArmMover()
#         # Move the arm up by 0.1 meters
#         arm_mover.move_arm_up(0.1)
#     except rospy.ROSInterruptException:
#         return
#     except KeyboardInterrupt:
#         return

# if __name__ == '__main__':
#     main()
