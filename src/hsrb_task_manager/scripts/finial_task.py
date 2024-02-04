#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

import tf

# define states
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
        # Return 'succeeded' or 'aborted' based on the outcome
        return 'succeeded'

class BagDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BagDetection')

        start_time = time.time()
        while time.time() - start_time < 5:  # 5 seconds timeout
            # Insert your bag detection logic here
            # If a bag is detected, return 'detected'
            if self.detect_bag():
                rospy.loginfo('Bag detected')
                return 'detected'

            rospy.sleep(0.1)  # Sleep for a short time to prevent high CPU usage

        rospy.loginfo('Bag not detected within 5 seconds')
        return 'not_detected'

    def detect_bag(self):
        # Implement your bag detection logic here
        # Return True if a bag is detected, otherwise False
        # For now, let's just return False as a placeholder
        return False
    
class BagGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BagGrasp')
        # Bag grasping logic here
        return 'succeeded'

class Remind_People_to_come(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Trying state PeopleDetection AGAIN')
        # People detection logic here
        return 'succeeded'

class PeopleDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state PeopleDetection')

        start_time = time.time()
        while time.time() - start_time < 5:  # 5 seconds timeout
            # Insert your people detection logic here
            # If a person is detected, return 'detected'
            if self.detect_person():
                rospy.loginfo('Person detected')
                return 'detected'

            rospy.sleep(0.1)  # Sleep for a short time to prevent high CPU usage

        rospy.loginfo('Person not detected within 5 seconds')
        return 'not_detected'

    def detect_person(self):
        # Implement your person detection logic here
        # Return True if a person is detected, otherwise False
        # For now, let's just return False as a placeholder
        return False
    
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation')
        # Navigation logic here
        return 'succeeded'

class PutDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PutDown')
        # Logic for putting down the bag
        return 'succeeded'
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

        smach.StateMachine.add('INIT_POSITION', InitPosition(), 
                               transitions={'succeeded':'BAG_DETECTION',
                                            'aborted':'aborted'})

        smach.StateMachine.add('BAG_DETECTION', BagDetection(), 
                               transitions={'detected':'BAG_GRASP',
                                            'not_detected':'INIT_POSITION'})

        smach.StateMachine.add('BAG_GRASP', BagGrasp(), 
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