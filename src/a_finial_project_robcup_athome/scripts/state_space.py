#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required modules
import rospy
import smach
import smach_ros
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header, String
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import quaternion_from_euler
import subprocess
import threading

# Your custom modules for specific robot capabilities
from stt_microphone import DeepSpeechRecognizer
from try_grasp_input import move_arm, move_gripper, move_torso, move_head
from tf_grasp import ArmController

def response_something(args):
    """
    Sends a text string to a ROS text-to-speech service to be spoken aloud by the robot.

    Parameters:
    - args: A string or command line arguments to be spoken by the robot.
            If args is empty, a default message is spoken instead.
    """
    # Split the input arguments string by spaces
    args = args.split()

    # Use the entire input as the text to speak if any is provided, otherwise use a default message
    text = " ".join(args) if args else "I don't have a command"

    rospy.loginfo("I'll say: " + text)

    # Connect to the ROS text-to-speech action server
    client = SimpleActionClient('/tts', TtsAction)
    client.wait_for_server()

    # Create a goal with the text to be spoken
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"  # Specify the language of the text

    # Send the goal to the server and wait for it to finish
    client.send_goal_and_wait(goal)


def recognize_speech(rate=16000, chunk=1024, record_seconds=5):
    """
    Records audio from the microphone for a given duration and recognizes speech using a DeepSpeech model.

    Parameters:
    - rate: The sample rate of the audio recording.
    - chunk: The size of each audio chunk to process.
    - record_seconds: The duration in seconds for which to record audio.

    Returns:
    - text: The recognized text from the recorded audio.
    """
    # Paths to the DeepSpeech model and scorer files
    MODEL_FILE = '/home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/config/deepspeech-0.9.3-models.pbmm'
    SCORER_FILE = '/home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/config/deepspeech-0.9.3-models.scorer'

    # Initialize the speech recognizer with the model, scorer, and recording parameters
    recognizer = DeepSpeechRecognizer(MODEL_FILE, SCORER_FILE, rate, chunk, record_seconds)

    try:
        # Record audio and perform speech recognition
        text = recognizer.record_and_recognize()
        return text
    finally:
        # Ensure the recognizer resources are released
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
        if True:  # Please replace it with actual conditional judgment logic
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
        if True:  # Please replace it with actual conditional judgment logic
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
                    response_something('please raise your arm to choose, which bag i should carry ')
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

class Find_Bag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.bag_goal = None
        self.bag_direction = None
        self.find_bag_sub = rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)
        self.bag_direction_sub = rospy.Subscriber("/arm_status", String, self.detect_direction_callback)
        self.attempt = 0
        self.bag_look_direction = None
        self.move_pose = [0.6,-0.95]
        
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
        if True:  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'
        
class BagGrasp(smach.State):
    def __init__(self,):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        rospy.Subscriber('/move_group/status', String, self.status_callback)
        rospy.Subscriber('/move_group/status', GoalStatusArray, self.status_callback)
        self.arm_controller = ArmController()
        self.look_right = [0.65,-0.98]
        self.look_left = [-0.65,-0.98]
        self.open_gripper = [0.04,0.04]
        self.close_gripper = [0.0,0.0]
        self.squat_up = 0.3
        self.squat_down = 0.07
        # self.stop_event = threading.Event()
        self.arm_style = None
        self.EndPosition_arm = [0.07, -1.00, -0.93, 1.89, -0.71, -1.11, 1.33]
        self.InitPosition_head = [0,0]
        self.InitPosition_torso = 0.2
        self.InitPosition_gripper = [0,0]

        self.move_status = False

    def status_callback(self, msg):
        status = msg.status_list
        for s in status:
            state = s.status 
            if state == 3:
                self.move_status = True
            else:
                self.move_status = False

    def grasp_bag(self):
        move_head([0, -0.98])
        rospy.sleep(10)
        self.move_status = False
        attempt = 0
        while True and attempt < 3:
            self.arm_controller.transform_and_move()
            if self.move_status:
                return True
            else:
                rospy.loginfo("Attempt to move arm failed, retrying...")
                attempt += 1
                rospy.sleep(5)  # Wait for 2 seconds before retrying
        return False
    
    def execute(self, userdata):
        rospy.loginfo('Executing state BagGrasp')

        move_gripper(self.open_gripper)
        # head_thread = self.move_head_continuously()
        
        rospy.sleep(5)
        if self.grasp_bag():
            rospy.loginfo('Robot succeeded to find bag')
            result = 'succeeded'
        else:
            rospy.loginfo('Robot failed to to find bag')
            result = 'aborted'
        return result

class BagCollect(smach.State):
    def __init__(self,):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.look_right = [0.65,-0.98]
        self.look_left = [-0.65,-0.98]
        self.open_gripper = [0.04,0.04]
        self.close_gripper = [0.0,0.0]
        self.squat_up = 0.3
        self.squat_down = 0.07
        self.arm_style = None
        self.EndPosition_arm = [0.07, -1.00, -0.93, 1.89, -0.71, -1.11, 1.33]
        self.InitPosition_head = [0,0]
        self.InitPosition_torso = 0.2
        self.InitPosition_gripper = [0,0]

        self.move_status = False

    def grasp_bag(self):
        try:
            move_torso(self.InitPosition_torso)
            rospy.sleep(3)
            move_torso(self.squat_down)
            rospy.sleep(5)
            move_gripper(self.close_gripper)
            rospy.sleep(3)
            move_torso(self.squat_up)
            rospy.sleep(3)
            move_arm(self.EndPosition_arm)
            rospy.sleep(2)
            response_something("Got it!")
            return True
        except:
            return False
    
    def execute(self, userdata):
        rospy.loginfo('Executing state BagGrasp')
        rospy.sleep(5)
        if self.grasp_bag():
            rospy.loginfo('Robot succeeded to find bag')
            result = 'succeeded'
        else:
            rospy.loginfo('Robot failed to to find bag')
            result = 'aborted'
        return result

class Look_Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.InitPosition_head = [0,0]

    def execute(self, userdata):
        move_head(self.InitPosition_head)
        rospy.loginfo('Executing state InitPosition')
        # Initialization logic her

        # Check some conditions to decide whether to return 'succeeded' or 'aborted'
        if True:  # Please replace it with actual conditional judgment logic
            rospy.loginfo('Robot succeeded to Init Position')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Init Position')
            return 'aborted'

class Move_to_Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Move to Human')
        try:
            response_something("I am now ready to follow you with your bag. Start moving away from me and I will follow. When you are at your destination, I would ask you to raise your hand to tell me to stop.")
            subprocess.call(['roslaunch', 'a_finial_project_robcup_athome', 'follow_human.launch'])
            rospy.loginfo('Robot succeeded to Move to Human')
            return 'succeeded'
        except:
            rospy.loginfo("FAILED TO LAUNCH")
            return 'aborted'
    
class PutDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.end_position = [0.4, -1.17, -1.9, 2.3, -1.3, -0.45, 1.75]
        self.open_gripper = [0.04,0.04]
    def execute(self, userdata):
        response_something('Here, you are .')
        move_arm(self.end_position)
        rospy.sleep(3)
        move_gripper(self.open_gripper)
        rospy.sleep(10)
        response_something(' wish you have a good day')
        return 'succeeded'
    
class ReturntoHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ReturntoHome')
        
        if ReturntoHome(): 
            subprocess.call(['rosrun', 'a_finial_project_robcup_athome', 'return_home.py'])
            rospy.loginfo('Robot succeeded to ReturntoHome')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to ReturntoHome')
            return 'aborted'

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

        smach.StateMachine.add('INIT_POSITION', InitPosition(), 
                               transitions={'succeeded':'Find_Human',
                                            'aborted':'aborted'})
        # smach.StateMachine.add('Find_Human', Find_Human(), 
        #                        transitions={'succeeded':'Arm_Dection',
        #                                     'aborted':'Remind_People_to_come'})
        smach.StateMachine.add('Find_Human', Find_Human(), 
                               transitions={'succeeded':'Look_Human',
                                            'aborted':'Remind_People_to_come'})
        smach.StateMachine.add('Remind_People_to_come', Remind_People_to_come(), 
                               transitions={'succeeded':'Find_Human'})
        
        # smach.StateMachine.add('Arm_Dection', Arm_Dection(), 
        #                        transitions={'succeeded':'Find_Bag',
        #                                     'aborted':'Find_Human'})
        # smach.StateMachine.add('Find_Bag', Find_Bag(), 
        #                        transitions={'succeeded':'Move_to_Bag',
        #                                     'aborted':'aborted'})
        # smach.StateMachine.add('Move_to_Bag', Move_to_Bag(), 
        #                        transitions={'succeeded':'BagGrasp',
        #                                     'aborted':'Bag_Helper'})
        # smach.StateMachine.add('Bag_Helper', Bag_Helper(), 
        #                        transitions={'succeeded':'Find_Bag',
        #                                     'aborted':'aborted'})       
        # smach.StateMachine.add('BagGrasp', BagGrasp(), 
        #                        transitions={'succeeded':'BagCollect',
        #                                     'aborted':'aborted'})
        
        # smach.StateMachine.add('BagCollect', BagCollect(), 
        #                        transitions={'succeeded':'Look_Human',
        #                                     'aborted':'aborted'})


        
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
        smach.StateMachine.add("ReturntoHome", ReturntoHome(), transitions={'succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()