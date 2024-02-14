#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

def read_waypoints(file_path):
    """
    Read waypoints from a file.

    Args:
        file_path (str): The path to the file containing the waypoints.

    Returns:
        list: A list of tuples, where each tuple contains the position and orientation of a waypoint.
    """
    waypoints = []
    with open(file_path, 'r') as file:
        for line in file:
            # Parse the position and orientation data from each line
            parts = line.split(',')
            position = Point(*map(float, [p.split(': ')[1] for p in parts[0:3]]))
            orientation = Quaternion(*map(float, [p.split(': ')[1] for p in parts[3:7]]))
            waypoints.append((position, orientation))
    return waypoints

def create_move_base_goal(position, orientation):
    """
    Create a MoveBaseGoal object.

    Args:
        position (Point): The position of the goal.
        orientation (Quaternion): The orientation of the goal.

    Returns:
        MoveBaseGoal: The MoveBaseGoal object.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = position
    goal.target_pose.pose.orientation = orientation
    return goal

def navigate_waypoints(waypoints):
    """
    Navigate through a list of waypoints.

    Args:
        waypoints (list): A list of tuples, where each tuple contains the position and orientation of a waypoint.
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    for position, orientation in reversed(waypoints):
        goal = create_move_base_goal(position, orientation)
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Failed to reach waypoint, stopping...")
            break
        else:
            rospy.loginfo("Waypoint reached. Moving to next...")

if __name__ == '__main__':
    try:
        rospy.init_node('rewind_navigation')
        waypoints_file_path = 'waypoints_record.txt'  # Adjust path as needed
        waypoints = read_waypoints(waypoints_file_path)
        if waypoints:
            navigate_waypoints(waypoints)
            rospy.signal_shutdown("Navigation finished")
        else:
            rospy.loginfo("No waypoints found.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
   
    