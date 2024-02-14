#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

def read_waypoints(file_path):
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
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = position
    goal.target_pose.pose.orientation = orientation
    return goal

def navigate_waypoints(waypoints):
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
   
    