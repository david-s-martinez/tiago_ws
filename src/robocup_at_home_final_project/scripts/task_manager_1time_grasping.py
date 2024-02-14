#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
import tf
from time import sleep
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty

# define states
class FindTargat(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted','succeeded'])
    # replace this part by your method
    def execute(self, userdata):
        rospy.loginfo('Executing state FindTargat')
        rospy.sleep(10) 
        #random.random() 来模拟识别过程，您应该用实际的物品识别逻辑替换这部分
        if random.random() < 1.0:  # 假设有50%的几率识别到物品
            rospy.loginfo('Target found')
            return 'succeeded'
        else:
            rospy.loginfo('Target not found')
            return 'aborted'
        
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

       # 初始化 MoveGroupCommander 和 SimpleActionClient
        self.body_planner = moveit_commander.MoveGroupCommander("arm_torso")
        self.target_pose = rospy.get_param("/target_pose")
        self.target_pose_1 = rospy.get_param("/target_pose_1")
        self.body_planner.set_planner_id("SBLkConfigDefault")
        self.body_planner.set_pose_reference_frame("base_footprint")

    def execute(self, userdata):
        rospy.loginfo('Executing state Grasp')
        # if not self.move_arm(arm_goal):
        #     rospy.logerr('Failed to move arm to grasp position')
        #     return 'aborted'
        self.move_arm(self.target_pose)
        rospy.sleep(1.0)  # 模拟抓取过程
        self.move_arm(self.target_pose_1)
        rospy.sleep(1.0)
        self.move_arm(self.target_pose)
        rospy.sleep(1.0)  # 模拟抓取过程
        rospy.loginfo('Object grasped successfully')
        return 'succeeded'

    def move_arm(self, goal):
        # Create a PoseStamped message from the input vector
        target_pose_msg = PoseStamped()
        target_pose_msg.header.frame_id = "base_footprint"
        target_pose_msg.header.stamp = rospy.Time.now()
        target_pose_msg.pose.position.x = goal[0]
        target_pose_msg.pose.position.y = goal[1]
        target_pose_msg.pose.position.z = goal[2]
        quaternion = tf.transformations.quaternion_from_euler(goal[3], goal[4], goal[5])
        target_pose_msg.pose.orientation.x = quaternion[0]
        target_pose_msg.pose.orientation.y = quaternion[1]
        target_pose_msg.pose.orientation.z = quaternion[2]
        target_pose_msg.pose.orientation.w = quaternion[3]

        # Set the target pose for the MoveGroupCommander
        self.body_planner.set_pose_target(target_pose_msg)

        rospy.loginfo("Planning to move {} to a target pose expressed in {}".format(
            self.body_planner.get_end_effector_link(), self.body_planner.get_planning_frame()))

        self.body_planner.set_start_state_to_current_state()
        self.body_planner.set_max_velocity_scaling_factor(1.0)

        # Create a motion plan
        motion_plan = self.body_planner.plan()
        print(len(motion_plan))
        # Set maximum time to find a plan
        self.body_planner.set_planning_time(5.0)

        # Execute the plan
        success = self.body_planner.go()


# main
def main():
    rospy.init_node('smach_example_state_machine')



    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    # Define user data for state machine
    sm.userdata.navGoalInd = 1
    # Open the container
    with sm:

        # Navigation callback
        def nav_cb(userdata, goal):
            navGoal = MoveBaseGoal()
            navGoal.target_pose.header.frame_id = "map"
            if userdata.navGoalInd == 1:
                rospy.loginfo('Navagate to table one')
                waypoint = rospy.get_param('/way_points/table_one')
                userdata.navGoalInd = 2
            elif userdata.navGoalInd == 2:
                rospy.loginfo('Navagate to table two')
                waypoint = rospy.get_param('/way_points/table_two')
                userdata.navGoalInd = 1
            navGoal.target_pose.pose.position.x = waypoint["x"]
            navGoal.target_pose.pose.position.y = waypoint["y"]
            navGoal.target_pose.pose.position.z = waypoint["z"]
            #navGoal.target_pose.pose.orientation.w = waypoint["w"]
            quaternion = tf.transformations.quaternion_from_euler(waypoint["roll"], waypoint["pitch"], waypoint["yaw"])
            navGoal.target_pose.pose.orientation.x = quaternion[0]
            navGoal.target_pose.pose.orientation.y = quaternion[1]
            navGoal.target_pose.pose.orientation.z = quaternion[2]
            navGoal.target_pose.pose.orientation.w = quaternion[3]
            return navGoal

        
        # Add states to the container and define the trasitions
        # Navigate to user defined waypoint with callback
        smach.StateMachine.add('NAVIGATION_TO_TABLE_ONE', 
                               smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'FIND_TARGET_ON_TABLE_ONE',
                                             'aborted':'aborted'})

        smach.StateMachine.add('NAVIGATION_TO_TABLE_TWO',
                                smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'FIND_TARGET_ON_TABLE_TWO',
                                            'aborted':'aborted'})

        smach.StateMachine.add('FIND_TARGET_ON_TABLE_ONE', FindTargat(), 
                                transitions={'succeeded':'GRASP', 
                                            'aborted':'NAVIGATION_TO_TABLE_TWO'})
        smach.StateMachine.add('FIND_TARGET_ON_TABLE_TWO', FindTargat(), 
                                transitions={'succeeded':'GRASP', 
                                            'aborted':'NAVIGATION_TO_TABLE_ONE'})
        smach.StateMachine.add('GRASP', Grasp(), 
                                transitions={'succeeded':'succeeded'})
    # Use a introspection for visulize the state machine
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()