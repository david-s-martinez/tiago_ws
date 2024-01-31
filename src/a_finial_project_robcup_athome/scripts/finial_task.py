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
from try_grasp import move_arm,move_gripper,move_torso
from try_moveit import ArmController

# define states
def some_condition_is_true():
    return True

def response_something(args):
    #rospy.init_node('say_something')
    command_line = args
    args = command_line.split()  
    # If the user adds some input, say what he wrote
    if len(args) >= 1:
        #使用" ".join(args[1:])来代替原来的循环拼接字符串
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
        # 录制并识别
        text = recognizer.record_and_recognize()
        return text
    finally:
        # 关闭资源
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
        # start_time = time.time()
        # while time.time() - start_time < 5:
            
        #     # Return 'succeeded' or 'aborted' based on the outcome
        #     return 'succeeded'
            
        # rospy.loginfo('Robot not succeeded to  Init Position')
        # return 'aborted'

        # 等待5秒
        rospy.sleep(5)

        # 检查某些条件，决定返回 'succeeded' 还是 'aborted'
        if some_condition_is_true():  # 请替换为实际的条件判断逻辑
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'
    
class BagDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BagDetection')
        response_something('Please, choose which bag i should carry')

        rospy.sleep(5)

        # 检查某些条件，决定返回 'succeeded' 还是 'aborted'
        if not some_condition_is_true():  # 请替换为实际的条件判断逻辑
            # Insert your bag detection logic here
            # If a bag is detected, return 'detected'

            if self.detect_bag(): 
                response_something('Do you want me to take this red one ')
                text = recognize_speech()
                print("Recognized Text:", text)
                if (text == 'yes' ):

                    rospy.loginfo('Bag detected')
                    return 'not_detected'# should change with detected

        else:
            rospy.loginfo('Bag not detected within 5 seconds')
            return 'detected'

    def detect_bag(self):
        # Implement your bag detection logic here
        # Return True if a bag is detected, otherwise False
        # For now, let's just return False as a placeholder
        return True
    
class BagGrasp(smach.State):
    def __init__(self,arm_controller):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.arm_controller = arm_controller
    def execute(self, userdata):
        rospy.loginfo('Executing state BagGrasp')
        response_something('Well, please let me grasp it')
        # moveit

        # 定义目标位置和姿态
        position = [0.4, 0.0, 0.4]  # X, Y, Z
        orientation = [0, -0.7, 0]  # R, P, Y (以弧度表示)

        # 移动手臂到指定位置
        self.arm_controller.move_arm(position, orientation)

        rospy.sleep(5)
        # Bag grasping logic here
        response_something('Well, it is too heavy , you might give me more tips')
        
        return 'succeeded'

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
        response_something('i just make sure you are you')

## do the detection
        rospy.sleep(5)
        # If a person is detected, return 'detected'
        if self.detect_person():
            rospy.loginfo('Person detected')
            return 'detected'
      
        else:
            rospy.loginfo('Person not detected within 5 seconds')
            return 'not_detected'

    def detect_person(self):
        # Implement your person detection logic here
        # Return True if a person is detected, otherwise False
        # For now, let's just return False as a placeholder
        return True
    
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        if some_condition_is_true():
            response_something('finially, we arrived.')
            rospy.loginfo('Executing state Navigation')
            # Navigation logic here
            return 'succeeded'
        else:
            rospy.loginfo('failed to state Navigation')
            # Navigation logic here
            return 'aborted'
        
class PutDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PutDown')
        response_something('Here, you are .')
        move_torso()
        move_gripper()
        move_arm()
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