#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction



def main():
    # Initialize the action client
    rospy.sleep(2.0)  # Wait for tf listener to initialize
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the server to start up
    client.wait_for_server()

    # Cancel all goals
    client.cancel_all_goals()
    
    HumanFollow()
    rospy.spin()

if __name__ == '__main__':
    main()