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
#from tiago_human_follow import HumanFollow
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header, String
from tiago_bag_follow import BagFollow
#from follow_point import LookToPoint 
import subprocess
from smach import State

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
        self.InitPosition_arm = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]
        self.InitPosition_head = [0,0]
        self.InitPosition_torso = 0.2
        self.InitPosition_gripper = [0,0]

    def InitialPosition(self):
        response_something('Hello, I am here to help you')
        move_head(self.InitPosition_head)
        move_arm(self.InitPosition_arm)
        move_gripper(self.InitPosition_gripper)
        move_torso(self.InitPosition_torso)
        return True

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic here

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if self.InitialPosition():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'

class Find_Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.find_human_sub = rospy.Subscriber('/goal_centroid', String, self.find_human_callback)
        self.human_goal = None
        self.attempt = 0
        
    def find_human_callback(self, msg):
        self.human_goal = msg.data
        
    def FindHuman(self):
        while self.human_goal is None:
            if self.attempt > 3:
                return False
            else:
                # say something
                response_something('please stand in front of me ')
                self.attempt += 1
                rospy.sleep(5)
                continue
        return True
    
    def execute(self, userdata):
        rospy.loginfo('Executing state FindHuman')

        if self.FindHuman(): 
            rospy.loginfo('Robot succeeded to Find Human')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Find Human')
            return 'aborted'
        
class Remind_People_to_come(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her
        response_something('i give you five seconds to let me see you  ')
        rospy.sleep(5)
        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'

        
class Remind_People_to_come_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her
        response_something('i give you five seconds to let me see you  ')
        rospy.sleep(5)
        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'

class Arm_Dection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.arm_dection_sub = rospy.Subscriber('/arm_status', String, self.arm_dection_callback)
        self.arm_style = None
        self.attempt = 0
        
    def arm_dection_callback(self, msg):
        self.arm_style = msg.data

    def arm_dection(self):
        while True:
            if self.attempt > 5:
                return False
            else:
                if self.arm_style in ['right', 'left']:
                    response_something(f'I see, you want me to take the {self.arm_style.lower()} one')
                    rospy.loginfo(f'arm detected {self.arm_style.lower()}')
                    rospy.sleep(5)
                    return True
                else:
                    # say something
                    response_something('please rise your arm to choose, which bag i should carry ')
                    self.attempt += 1
                    rospy.sleep(5)


    def execute(self, userdata):
        rospy.loginfo('Executing state arm_dection')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if self.arm_dection():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to arm_dection')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to arm_dection')
            return 'aborted'

# class Find_Bag(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
#         rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)
#         self.found_bag = False

#     def detect_bag_callback(self, msg):
#         if msg.data != "None":
#             response_something('find bag')
#             rospy.loginfo("find ")
#             self.found_bag = True
    
#     def Find_Bag(self):
#         while not self.found_bag:
#             response_something('not find')
#             rospy.sleep(1)  
#             return False

#     def execute(self, userdata):
#         rospy.loginfo('Executing state Find_Bag')


#         # Check some conditions to decide whether to return 'succeeded' or'aborted'
#         while not self.Find_Bag():
#             if 
#             rospy.loginfo('Robot succeeded to Init Position')
#             return 'succeeded'
#         else:
#             rospy.loginfo('Robot not succeeded to Init Position')
#             return 'aborted'

class Find_Bag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.bag_goal = None
        self.bag_direction = None
        self.find_bag_sub = rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)
        self.bag_direction_sub = rospy.Subscriber("/arm_status", String, self.detect_direction_callback)
        self.attempt = 0
        self.bag_look_direction = None
        self.move_pose = [0.7,-0.98]
        #self.LookToPoint = LookToPoint()
        
    def detect_bag_callback(self, msg):
        self.bag_goal = msg.data
        if self.bag_goal is not None:
            self.bag_found = True
        else:
            self.bag_found = False
        
    def detect_direction_callback(self, msg):
        self.bag_direction = msg.data
        if self.bag_direction == "left":
            self.bag_look_direction = -1.0
        elif self.bag_direction == "right":
            self.bag_look_direction = 1.0
            
    def continuous_move_head(self, look_direction, stop_event, interval=1.0):
        while not stop_event.is_set():
            move_head(look_direction)
            rospy.sleep(interval)
            
    def move_head(self):
        self.stop_event = threading.Event()
        look_direction = [self.move_pose[0] * self.bag_look_direction, self.move_pose[1]]
        self.head_thread = threading.Thread(target=self.continuous_move_head, args=(look_direction, self.stop_event))
        self.head_thread.start()
    
    def FindBag(self):
        if self.bag_direction is not None:
            response_something("I am looking for the bag on the %s side" % self.bag_direction)      
        self.move_head()
        #self.LookToPoint
        while not self.bag_found:
            if self.attempt > 3:
                return False
            else:
                self.attempt += 1
        self.stop_event.set()
        self.head_thread.join()
        return True

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        if self.FindBag(): 
            rospy.loginfo('Robot succeeded to Find Bag')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Find Bag')
            return 'aborted'
        
class Move_to_Bag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MovetoBag')
        try:
            subprocess.call(['roslaunch', 'a_finial_project_robcup_athome', 'follow_bag.launch'])
            rospy.loginfo('Robot succeeded to MovetoBag')
            return 'succeeded'
        except:
            rospy.loginfo("FAILED TO LAUNCH")
            return 'aborted'
        
class Bag_Helper(smach.State):
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
    def __init__(self,):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.arm_controller = ArmController()
        self.look_right = [0.7,-0.98]
        self.look_left = [-0.7,-0.98]
        self.open_gripper = [0.04,0.04]
        self.close_gripper = [0.0,0.0]
        self.squat_up = 0.3
        self.squat_down = 0.13
        # self.stop_event = threading.Event()
        self.arm_style = None

    # def move_head_continuously(self, direction):
    #     if direction == "right":
    #         look_direction = self.look_right  
    #     elif direction == "left":
    #         look_direction = self.look_left
    #     head_thread = threading.Thread(target=self.continuous_move_head, args=(look_direction, self.stop_event))
    #     head_thread.start()
    #     return head_thread

    # def continuous_move_head(self, look_direction, stop_event, interval=1.0):
    #     while not stop_event.is_set():
    #         move_head(look_direction)
    #         rospy.sleep(interval)

    def grasp_bag(self):
        arm_moved_successfully = False
        attempt = 0
        while not arm_moved_successfully and attempt < 3:
            arm_moved_successfully = self.arm_controller.transform_and_move()
            if not arm_moved_successfully:
                rospy.loginfo("Attempt to move arm failed, retrying...")
                attempt += 1
                rospy.sleep(2)  # Wait for 2 seconds before retrying
        return arm_moved_successfully

    def execute(self, userdata):
        rospy.loginfo('Executing state BagGrasp')

        move_gripper(self.open_gripper)
        # head_thread = self.move_head_continuously()
        
        rospy.sleep(5)
        if self.grasp_bag():
            rospy.loginfo('Robot succeeded to grasp the bag')
            result = 'succeeded'
            move_torso(self.squat_down)
            rospy.sleep(5)
            move_gripper(self.close_gripper)
            rospy.sleep(3)
            move_torso(self.squat_up)
        else:
            rospy.loginfo('Robot failed to grasp the bag')
            result = 'aborted'
        return result

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
        self.finish_navigation = rospy.Publisher("/human_finish", String, queue_size = 1)
        self.finish_navigation.publish("Not at Goal")

    def execute(self, userdata):
        rospy.loginfo('Executing state Move to Human')
        try:
            response_something('I am now ready to follow you with your bag. Start moving away from me and I will follow.')
            subprocess.call(['roslaunch', 'a_finial_project_robcup_athome', 'follow_human.launch'])
            # subprocess.call(['rosrun', 'a_finial_project_robcup_athome', 'tiago_human_follow.py'])
            rospy.loginfo('Robot succeeded to Move to Human')
            return 'succeeded'
        except:
            rospy.loginfo("FAILED TO LAUNCH")
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
                                            'aborted':'Remind_People_to_come'})
        smach.StateMachine.add('Remind_People_to_come', Remind_People_to_come(), 
                               transitions={'succeeded':'Find_Human'})
        
        smach.StateMachine.add('Arm_Dection', Arm_Dection(), 
                               transitions={'succeeded':'Find_Bag',
                                            'aborted':'Find_Human'})
        smach.StateMachine.add('Find_Bag', Find_Bag(), 
                               transitions={'succeeded':'Move_to_Bag',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Move_to_Bag', Move_to_Bag(), 
                               transitions={'succeeded':'BagGrasp',
                                            'aborted':'Bag_Helper'})
        smach.StateMachine.add('Bag_Helper', Bag_Helper(), 
                               transitions={'succeeded':'Find_Bag',
                                            'aborted':'aborted'})       
        smach.StateMachine.add('BagGrasp', BagGrasp(), 
                               transitions={'succeeded':'Look_Human',
                                            'aborted':'aborted'})
        smach.StateMachine.add('Look_Human', Look_Human(), 
                               transitions={'succeeded':'Move_to_Human',
                                            'aborted':'Remind_People_to_come_2'})
        smach.StateMachine.add('Remind_People_to_come_2', Remind_People_to_come_2(), 
                               transitions={'succeeded':'Look_Human'})
        smach.StateMachine.add('Move_to_Human', Move_to_Human(), 
                               transitions={'succeeded':'PUT_DOWN',
                                            'aborted':'Look_Human'})
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