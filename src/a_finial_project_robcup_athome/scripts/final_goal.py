#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal

class RobotController:
    def __init__(self):
        rospy.init_node('robot_waypoint_navigator', anonymous=True)
        # self.waypoints = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback)
        self.arm_dection_sub = rospy.Subscriber('/arm_status', String, self.arm_status_callback)
        self.finish_pub = rospy.Publisher("/navigation_finished", String, queue_size=1)

        self.arm_raised = False
        self.current_waypoint = None

    # def goal_callback(self, msg):
    #     goal_position = msg.goal.target_pose.pose.position
    #     goal_orientation = msg.goal.target_pose.pose.orientation
    #     # Format the data as a string or a structured record
    #     waypoint_data = "Goal - Position (x: {}, y: {}, z: {}), Orientation (x: {}, y: {}, z: {}, w: {})\n".format(
    #         goal_position.x, goal_position.y, goal_position.z,
    #         goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w)
    
    # Assuming you have a function to handle file operations
        # self.write_to_file("waypoints_record.txt", waypoint_data)

    # def write_to_file(filename, data):
    #     with open(filename, "a") as file:
    #         file.write(data)

    def arm_status_callback(self, msg):
        self.arm_raised = msg.data
        if self.arm_raised is not None:
            rospy.loginfo("Arm raised detected, stopping and finishing navigation.")
            self.finish_pub.publish("finish")
            rospy.signal_shutdown("Navigation finished due to arm raised signal.")

if __name__ == "__main__":
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
