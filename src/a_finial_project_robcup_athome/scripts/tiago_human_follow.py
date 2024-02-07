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

class RobotFollow:
    def __init__(self):
        self.goal_pose = None
        self.goal_ori = None
        self.goal_init = False
        self.replanning_attempts = 0
        self.max_replanning_attempts = 3 

        rospy.init_node(NODE_NAME, anonymous=True)
        self.client = actionlib.SimpleActionClient(MOVE_BASE_ACTION_SERVER, MoveBaseAction)
        self.client.wait_for_server()

        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.pose_callback)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.status_callback)
        
        self.listener = tf.TransformListener()
        rospy.loginfo("f{NODE_NAME} initialized")
        self.human_status = rospy.Publisher(FOLLOW_STATUS, String, queue_size=1)

    def pose_callback(self, msg):
        self.goal_pose = msg.pose.position
        self.goal_ori = msg.pose.orientation
        self.goal_init = True
        self.create_goal_path()
        self.replanning_attempts = 0
        self.create_goal_path()

    # create a status publisher to publish the status of the robot
    def status_callback(self, msg):
        if msg.status.status == actionlib.GoalStatus.ABORTED:
            self.replanning_attempts += 1
            rospy.loginfo("Replanning attempt: %d" % self.replanning_attempts)
            if self.replanning_attempts >= self.max_replanning_attempts:
                self.cancel_goal()
                rospy.loginfo("Goal aborted, replanning attempts exceeded")
                self.human_status.publish(String("Failed"))
        if msg.status.status == actionlib.GoalStatus.SUCCEEDED:
            self.human_status.publish(String("Success"))
        else:
            self.human_status.publish(String("Moving"))
        
    def cancel_goal(self):
        self.client.cancel_all_goals()
        rospy.loginfo("Goal cancelled")

    def create_goal_path(self):
        if self.goal_init: 
            try:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = MAP_FRAME_ID
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position = self.goal_pose
                goal.target_pose.pose.orientation = self.goal_ori
                self.client.send_goal(goal)
                rospy.loginfo("Goal point sent to move_base")
                self.client.wait_for_result()
            except Exception as e:
                rospy.logerr("Error in sending goal point to move_base: %s" % e)

def main():
    rospy.sleep(2.0)  # Wait for the tf listener to initialize
    try:
        RobotFollow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
