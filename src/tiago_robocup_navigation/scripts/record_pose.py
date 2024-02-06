#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import threading

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower', anonymous=True)
    
        self.human_position_topic = "/human_goal"
        rospy.Subscriber(self.human_position_topic, PoseStamped, self.human_position_callback)
        
        # This list will store the waypoints
        self.waypoints = []
        
        # Lock for thread-safe operations on waypoints list
        self.lock = threading.Lock()
        
        # Subscribe to the topic where the human's position is published.
        # This is an example topic; you should replace it with the actual topic from your setup.

        # Publisher to publish waypoints for the robot to follow.
        # You might need to adapt this to fit your robot's navigation system.
        self.waypoint_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=10)

    def human_position_callback(self, msg):
        # Callback function that gets called every time a new message is received from the human position topic.
        with self.lock:
            self.waypoints.append(msg)

    def follow_waypoints_reverse(self):
        # This function reverses the waypoints and makes the robot follow them one by one.
        rospy.loginfo("Following waypoints in reverse order.")
        with self.lock:
            reverse_waypoints = list(reversed(self.waypoints))
        
        for waypoint in reverse_waypoints:
            # Publish each waypoint to the robot's navigation system to make it move to that point.
            # You might need to add delays or checks here to wait for the robot to reach each waypoint.
            self.waypoint_pub.publish(waypoint)
            rospy.sleep(1)  # Simple delay, adjust based on your robot's speed and accuracy needs.

    def run(self):
        # Main loop of the script
        rospy.loginfo("Waypoint follower is running.")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down waypoint follower.")
            self.follow_waypoints_reverse()

if __name__ == '__main__':
    waypoint_follower = WaypointFollower()
    waypoint_follower.run()
