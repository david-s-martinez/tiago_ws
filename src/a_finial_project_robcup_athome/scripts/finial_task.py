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

class FindTargat(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted','succeeded'])
    # replace this part by your method
    def execute(self, userdata):
        rospy.loginfo('Executing state FindTargat')
        if random.random() < 0.9:
            rospy.loginfo('Target not found')
            return 'aborted'
        else:
            rospy.loginfo('Target found')
            return 'succeeded'
        
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Grasp')
        return 'succeeded'

class InitPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic here
        rospy.loginfo('saying')
        
        response_something('Hello, I am here to help you')
        InitPosition_arm = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]
        InitPosition_head = [0,0]
        InitPosition_torso = 0.2
        InitPosition_gripper = [0,0]
        move_head(InitPosition_head)
        move_arm(InitPosition_arm)
        move_gripper(InitPosition_gripper)
        move_torso(InitPosition_torso)
        # start_time = time.time()
        # while time.time() - start_time < 5:
            
        #     # Return 'succeeded' or 'aborted' based on the outcome
        #     return 'succeeded'
            
        # rospy.loginfo('Robot not succeeded to  Init Position')
        # return 'aborted'

        # wait 5 seconds
        rospy.sleep(5)

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if some_condition_is_true():  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'
    
class BagDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'])
        self.hand_detector = HandDetector()  # 创建手部识别器实例
        self.hand_direction = "" # 存储手部方向的变量
        rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)
        self.found_bag = False
        
    def detect_bag_callback(self, msg):
        if msg.data != "None":
            response_something('find bag')
            rospy.loginfo("find ")
            self.found_bag = True
        # else:
        #     response_something('find bag')
        #     rospy.loginfo("find: " + msg.data)
        #     self.found_bag = True

    def execute(self, userdata):
        rospy.loginfo('Executing state BagDetection')
        response_something('Please, choose which bag i should carry')
        while not rospy.is_shutdown() and not self.found_bag:
            response_something('not find')
            rospy.sleep(1)  # 短暂休眠以避免CPU过载

        if rospy.is_shutdown():
            rospy.loginfo('not_detected')
            return 'not_detected'
        elif self.found_bag:
            rospy.loginfo('detected')
            return 'detected'  
        #rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)
        #rospy.sleep(50)
        # while not self.found_bag:
        #     rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)
        #     rospy.sleep(3)
        #     if self.found_bag:
        #         return 'detected'
        # else:
        #     return 'not_detected'

        # # 设定超时时间，例如60秒
        # timeout = rospy.Duration(60)
        # start_time = rospy.Time.now()

        # rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)

        # while rospy.Time.now() - start_time < timeout:
        #     rospy.loginfo('into while')
        #     self.detect_bag()
        #     print(self.hand_direction)

        #     if self.detect_bag():
        #         #self.hand_direction = self.hand_detector.print_direction
        #         response_something(f'I see, you want me to take the {self.hand_direction.lower()} one')
        #         rospy.loginfo('Bag detected')
        #         return 'detected'

        #     # 没有检测到，等待10秒后再次检测
        #     rospy.sleep(10)

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        # if some_condition_is_true():  # Please replace it with actual conditional judgment logic
        #     # Insert your bag detection logic here
        #     # If a bag is detected, return 'detected'
        #     if self.detect_bag(): 
        #         response_something('i see , you want me to take right one ')


        #         rospy.loginfo('Bag detected')
        #         return 'detected'# should change with detected
        #     # if self.detect_bag(): 
        #     #     response_something('Do you want me to take this red one ')
        #     #     text = recognize_speech()
        #     #     print("Recognized Text:", text)
        #     #     if (text == 'yes' ):

        #     #         rospy.loginfo('Bag detected')
        #     #         return 'not_detected'# should change with detected

        # else:
        #     rospy.loginfo('Bag not detected within 5 seconds')
        #     return 'not_detected'

    def detect_bag(self):
        # Implement your bag detection logic here
        self.hand_direction = self.hand_detector.direction_text
        global hand_command
        hand_command = self.hand_direction
        if self.hand_direction == 'RIGHT':
            # 如果检测到右手，返回 True
            rospy.loginfo('Bag detected right')
            return True
        elif self.hand_direction == 'LEFT':
            # 如果检测到左手，返回 True
            rospy.loginfo('Bag detected left ')
            return True
        # 如果没有检测到手或者方向不是左或右，返回 False
        return False
    
class BagGrasp(smach.State):
    def __init__(self,arm_controller):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.arm_controller = arm_controller

    def execute(self, userdata):
        rospy.loginfo('Executing state BagGrasp')
        response_something('Well, please let me grasp it')
        #head_commdand = "Right"
        look_right = [0.7,-0.98]
        look_left = [-0.7,-0.98]
        open_gripper = [0.04,0.04]
        close_gripper = [0.0,0.0]
        move_gripper(open_gripper)
        global hand_command
        head_command = hand_command
        stop_event = threading.Event()
        if head_command == "Right":
            look_direction = look_right  # 或 look_left = [-0.7, -0.8]
        elif head_command == "Left":
            look_direction = look_left
        head_thread = threading.Thread(target=self.continuous_move_head, args=(look_direction, stop_event))
        head_thread.start()

        rospy.sleep(5) 
        arm_moved_successfully = False
        while not arm_moved_successfully:
            arm_moved_successfully = self.arm_controller.transform_and_move()
            if not arm_moved_successfully:
                rospy.loginfo("Attempt to move arm failed, retrying...")
                response_something('i cant touch it , i will try it again')
                rospy.sleep(30)  # Wait for 2 seconds before retrying

        rospy.sleep(10)

        squat_down = 0.13
        move_torso(squat_down)
        rospy.sleep(5)

        move_gripper(close_gripper)
        rospy.sleep(3)
        InitPosition_arm = [0.4, -1.17, -1.9, 2.3, -1.3, -0.45, 1.75]
        squat_up = 0.3
        move_torso(squat_up)
        rospy.sleep(5)
        # move_arm(InitPosition_arm)
        # 停止持续移动头部的线程
        stop_event.set()
        head_thread.join()

        # Bag grasping logic here
        response_something('Well, it is too heavy , you might give me more tips')
        
        return 'succeeded'
    
    def continuous_move_head(self,look_direction, stop_event, interval=1.0):
        while not stop_event.is_set():
            move_head(look_direction)
            rospy.sleep(interval)


class Remind_People_to_come(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Trying state PeopleDetection AGAIN')
        response_something('please, stand inside my view')
        # People detection logic here
        return 'succeeded'

class PeopleDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state PeopleDetection')
        response_something('come close , i just make sure you are you')

        ## do the detection
        rospy.sleep(5)
        # If a person is detected, return 'detected'
        if self.detect_person():
            rospy.loginfo('Person detected')
            response_something('i see you ,i will begin to follow you')
            return 'detected'
      
        else:
            rospy.loginfo('Person not detected within 5 seconds')
            return 'not_detected'

    def detect_person(self):
        # self.pick_centroid_pub = rospy.Publisher('/pick_centroid', String, queue_size=10)
        #rospy.Subscriber("/pick_centroid", String, self.point_callback)
        # rospy.Subscriber("/goal_point", PointStamped, self.point_callback)
        # Implement your person detection logic here
        # Return True if a person is detected, otherwise False
        # For now, let's just return False as a placeholder
        return True
    
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation')
        target_point_getter = GetTargetPoint()

        while not rospy.is_shutdown() and not target_point_getter.has_valid_point:
            rospy.sleep(0.1)  

        if rospy.is_shutdown():
            return 'aborted'

        response_something('Finally, we arrived.')
        return 'succeeded'

        # if some_condition_is_true():
            
        #     rospy.loginfo('Executing state Navigation')
        #     response_something('finially, we arrived.')
        #     # Navigation logic here
        #     return 'succeeded'
        # else:
        #     rospy.loginfo('failed to state Navigation')
        #     # Navigation logic here
        #     return 'aborted'
        
class PutDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PutDown')
        response_something('Here, you are .')
        end_position = [0.4, -1.17, -1.9, 2.3, -1.3, -0.45, 1.75]
        move_arm(end_position)
        rospy.sleep(10)
        response_something(' wish you have a good day')
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
                               transitions={'succeeded':'BAG_DETECTION',
                                            'aborted':'aborted'})

        smach.StateMachine.add('BAG_DETECTION', BagDetection(), 
                               transitions={'detected':'BAG_GRASP',
                                            'not_detected':'INIT_POSITION'})

        smach.StateMachine.add('BAG_GRASP', BagGrasp(arm_controller), 
                               transitions={'succeeded':'PEOPLE_DETECTION',
                                            'aborted':'BAG_DETECTION'})

        smach.StateMachine.add('PEOPLE_DETECTION', PeopleDetection(), 
                               transitions={'detected':'NAVIGATION',
                                            'not_detected':'Remind_People_to_come'})

        smach.StateMachine.add('Remind_People_to_come', Remind_People_to_come(), 
                               transitions={'succeeded':'PEOPLE_DETECTION'})
        
        smach.StateMachine.add('NAVIGATION', Navigation(), 
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