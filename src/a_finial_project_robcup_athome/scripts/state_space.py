#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import smach
import smach_ros
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import tf
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from stt_microphone import DeepSpeechRecognizer
from try_grasp_input import move_arm,move_gripper,move_torso,move_head
from tf_grasp import ArmController
import threading
from hands_command import HandDetector
from create_goal import GetTargetPoint
from tiago_human_follow import HumanFollow
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header, String

# define states
def some_condition_is_true():
    return True

def response_something(args):
    #rospy.init_node('say_something')
    command_line = args
    args = command_line.split()  
    # If the user adds some input, say what he wrote
    if len(args) >= 1:
        #Use " ".join(args[1:]) to replace the original loop splicing string
        text = " ".join(args[0:])
    # If not, just say a sentence
    else:
        text = "I don't got command"

    rospy.loginfo("I'll say: " + text)
    
    # Connect to the text-to-speech action server
    client = SimpleActionClient('/tts', TtsAction)
    client.wait_for_server()
    
    # Create a goal to say our sentence
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"
    
    # Send the goal and wait
    client.send_goal_and_wait(goal)

def recognize_speech( rate=16000, chunk=1024, record_seconds=5):
    MODEL_FILE = '/home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/config/deepspeech-0.9.3-models.pbmm'
    SCORER_FILE = '/home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/config/deepspeech-0.9.3-models.scorer'

    recognizer = DeepSpeechRecognizer(MODEL_FILE, SCORER_FILE, rate, chunk, record_seconds)
    try:
        # Record and recognize
        text = recognizer.record_and_recognize()
        return text
    finally:
        # Close resource
        recognizer.close()


class InitPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic here
        rospy.loginfo('saying')
        

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'

class Find_Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'
        
class Arm_Dection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'

class Find_Bag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'

class Move_to_Bag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'
    
class BagGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'

class Look_Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'

class Move_to_Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'
    
class PutDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        
        return 'succeeded'
    

# main
def main():
    rospy.init_node('smach_example_state_machine')
    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    # Define user data for state machine
    sm.userdata.navGoalInd = 1
    # Open the container
    arm_controller = ArmController()
    #bag_grasp_state = BagGrasp(arm_controller)
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

        smach.StateMachine.add('INIT_POSITION', InitPosition(), 
                               transitions={'succeeded':'Find_Human',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Find_Human', Find_Human(), 
                               transitions={'succeeded':'Arm_Dection',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Arm_Dection', Arm_Dection(), 
                               transitions={'succeeded':'Find_Bag',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Find_Bag', Find_Bag(), 
                               transitions={'succeeded':'Move_to_Bag',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Move_to_Bag', Move_to_Bag(), 
                               transitions={'succeeded':'BagGrasp',
                                            'aborted':'aborted'})
        smach.StateMachine.add('BagGrasp', BagGrasp(), 
                               transitions={'succeeded':'Look_Human',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Look_Human', Look_Human(), 
                               transitions={'succeeded':'Move_to_Human',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Move_to_Human', Move_to_Human(), 
                               transitions={'succeeded':'PUT_DOWN',
                                            'aborted':'aborted'})
        smach.StateMachine.add('PUT_DOWN', PutDown(), 
                           transitions={'succeeded':'succeeded'
                                        # ,'aborted':'aborted'
                                        })

        # Add states to the container and define the trasitions
        # Navigate to user defined waypoint with callback
        # smach.StateMachine.add('NAVIGATION_TO_TABLE_ONE', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
        #                         transitions={'succeeded':'FIND_TARGET_ON_TABLE_ONE',
        #                                      'aborted':'aborted'})

        # smach.StateMachine.add('NAVIGATION_TO_TABLE_TWO', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
        #                         transitions={'succeeded':'FIND_TARGET_ON_TABLE_TWO',
        #                                     'aborted':'aborted'})

        # smach.StateMachine.add('FIND_TARGET_ON_TABLE_ONE', FindTargat(), 
        #                         transitions={'succeeded':'GRASP', 
        #                                     'aborted':'NAVIGATION_TO_TABLE_TWO'})
        # smach.StateMachine.add('FIND_TARGET_ON_TABLE_TWO', FindTargat(), 
        #                         transitions={'succeeded':'GRASP', 
        #                                     'aborted':'NAVIGATION_TO_TABLE_ONE'})
        # smach.StateMachine.add('GRASP', Grasp(), 
        #                         transitions={'succeeded':'succeeded'})
    # Use a introspection for visulize the state machine
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()