#!/usr/bin/env python3
import rospy
import moveit_commander
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import sys

class ArmController:
    def __init__(self):
        # 初始化MoveIt! Commander和ROS节点
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('tiago_arm_controller', anonymous=True)

        # 初始化MoveGroupCommander对象
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")

        # 设置参考坐标系
        self.arm_group.set_pose_reference_frame("base_footprint")

    def move_arm(self, position, orientation):
        # 创建目标姿态
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]
        
        # 将RPY转换为四元数
        quaternion = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        # 设置目标姿态并规划运动
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()

        # 执行
        self.arm_group.go(wait=True)

        # 清除目标姿态
        self.arm_group.clear_pose_targets()

    def move_arm_joint_space(self, joint_goals):
        print(len(self.arm_group.get_active_joints()))
        # 创建JointState消息
        joint_state = JointState()
        joint_state.name = ['torso_lift_joint','arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        joint_state.position = joint_goals

        # 设置关节角度目标
        self.arm_group.set_joint_value_target(joint_state)

        # 规划并执行
        self.arm_group.go(wait=True)

        #清除目标
        self.arm_group.clear_pose_targets()

    def set_target_from_rviz(self):
        rospy.Subscriber("/goal", PoseStamped, self.callback_pose_goal)
        #rospy.Subscriber("/interactive_marker_pose/update", PoseStamped, self.callback_pose_goal)

    def callback_pose_goal(self, msg):
        # 设置目标姿态并规划运动
        msg.header.stamp = rospy.Time.now()
        self.arm_group.set_pose_target(msg)
        self.arm_group.go(wait=True)
        self.arm_group.clear_pose_targets()

def main():
    arm_controller = ArmController()

    # 定义目标位置和姿态
    position = [0.4, 0.0, 0.4]  # X, Y, Z
    orientation = [0, -0.7, 0]  # R, P, Y (以弧度表示)

    # 移动手臂到指定位置
    #arm_controller.move_arm(position, orientation)

    # # 定义关节目标位置（根据您的机器人关节数量和名称进行调整）
    # joint_goals = [0.1,0.07, -0.5, -0.3, 2.13, -1.33, -0.43, -0.2] # 替换为适当的目标值

    # # 移动手臂到指定关节位置
    # arm_controller.move_arm_joint_space(joint_goals)
    arm_controller.set_target_from_rviz()
    rospy.spin()



if __name__ == '__main__':
    main()
