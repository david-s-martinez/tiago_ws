#!/usr/bin/env python3
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from std_msgs.msg import String

# Constants for topic names and frame ids
NODE_NAME = "human_follow"
GOAL_TOPIC = "/human_goal"
ROBOT_POSE_TOPIC = "/robot_pose"
MOVE_BASE_ACTION_SERVER = "move_base"
MAP_FRAME_ID = "map"
BASE_FOOTPRINT_FRAME_ID = "base_footprint"
FOLLOW_STATUS = "/human_status"

class HumanFollow:
    def __init__(self):
        self.goal_pose = None
        self.goal_ori = None
        self.goal_init = False
        self.replanning_attempts = 0
        self.max_replanning_attempts = 5
        self.status = ""
        self.arm_raised = False
        self.move = False

        rospy.init_node(NODE_NAME, anonymous=True)
        self.client = actionlib.SimpleActionClient(MOVE_BASE_ACTION_SERVER, MoveBaseAction)
        self.client.wait_for_server()
        
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.status_callback)
        self.goal_sub = rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.pose_callback)
        self.human_status = rospy.Publisher(FOLLOW_STATUS, String, queue_size=1)
        rospy.sleep(5)
        
        self.arm_dection_sub = rospy.Subscriber('/arm_status', String, self.arm_status_callback)
        
        rospy.sleep(5)

        self.listener = tf.TransformListener()
        rospy.loginfo("f{NODE_NAME} initialized")

    def arm_status_callback(self, msg):
        arm_raised = msg.data
        if arm_raised == "left" or arm_raised == "right":
            self.arm_raised = True
            rospy.loginfo("Arm raised detected, stopping and finishing navigation.")
            rospy.signal_shutdown("Navigation finished due to arm raised signal.")
        else:
            self.arm_raised = False
            self.create_goal_path()

    def pose_callback(self, msg):
        self.goal_pose = msg.pose.position
        self.goal_ori = msg.pose.orientation

    # create a status publisher to publish the status of the robot
    def status_callback(self, msg):
        self.status = msg.status.status
        rospy.loginfo(self.status)
        
    def cancel_goal(self):
        self.client.cancel_all_goals()
        rospy.loginfo("Goal cancelled")

    def create_goal_path(self):
        try:
            if self.arm_raised:
                self.cancel_goal()
                return
            if self.replanning_attempts > self.max_replanning_attempts:
                rospy.signal_shutdown("Navigation finished due to arm raised signal.")

            if self.goal_pose.x == None:
                return
            
            if self.move == False:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = MAP_FRAME_ID
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position = self.goal_pose
                goal.target_pose.pose.orientation = self.goal_ori
            
                rospy.loginfo("Goal point sent to move_base")
                self.client.send_goal(goal)

            if self.status == 2:
                self.move = True
            
            if self.status == 3:
                rospy.loginfo("At Goal")
                rospy.sleep(1)
                self.replanning_attempts += 1
                self.move = False
            elif self.status == 4 or self.status == 5:
                self.cancel_goal()
                self.move = False

        except Exception as e:
            rospy.logerr("Error in sending goal point to move_base: %s" % e)

def main():
    rospy.sleep(2.0)  # Wait for the tf listener to initialize
    try:
        HumanFollow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
