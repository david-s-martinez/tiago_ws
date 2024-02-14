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
