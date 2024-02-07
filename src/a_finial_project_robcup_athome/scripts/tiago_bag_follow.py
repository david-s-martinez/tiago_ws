#!/usr/bin/env python3
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from std_msgs.msg import String

# Constants for topic names and frame ids
NODE_NAME = "bag_follow"
GOAL_TOPIC = "/bag_goal"
ROBOT_POSE_TOPIC = "/robot_pose"
MOVE_BASE_ACTION_SERVER = "move_base"
MAP_FRAME_ID = "map"
BASE_FOOTPRINT_FRAME_ID = "base_footprint"
FOLLOW_STATUS = "/bag_status"

class BagFollow:
    def __init__(self):
        self.goal_pose = None
        self.goal_ori = None
        self.robot_pose = None
        self.robot_ori = None
        self.replanning_attempts = 0
        self.max_replanning_attempts = 1

        rospy.init_node(NODE_NAME, anonymous=True)
        self.client = actionlib.SimpleActionClient(MOVE_BASE_ACTION_SERVER, MoveBaseAction)
        self.client.wait_for_server()
        
        rospy.Subscriber(ROBOT_POSE_TOPIC, PoseStamped, self.robot_pose_callback)

        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.pose_callback)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.status_callback) 
        
        self.listener = tf.TransformListener()
        self.bag_status = rospy.Publisher(FOLLOW_STATUS, String, queue_size=1)
        rospy.loginfo("f{NODE_NAME} initialized")

    def pose_callback(self, msg):
        self.goal_pose = msg.pose.position
        self.goal_ori = msg.pose.orientation
        self.replanning_attempts = 0
        self.create_goal_path()
        
    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose.position
        self.robot_ori = msg.pose.orientation
        self.create_goal_path()

    # create a status publisher to publish the status of the robot
    def status_callback(self, msg):
        if msg.status.status == actionlib.GoalStatus.ABORTED:
            self.replanning_attempts += 1
            rospy.loginfo("Replanning attempt: %d" % self.replanning_attempts)
            if self.replanning_attempts >= self.max_replanning_attempts:
                self.cancel_goal()
                self.bag_status.publish(String("Failed"))
                rospy.loginfo("Goal aborted, replanning attempts exceeded")
        if msg.status.status == actionlib.GoalStatus.SUCCEEDED:
            self.bag_status.publish(String("Success"))
        
    def cancel_goal(self):
        self.client.cancel_all_goals()
        rospy.loginfo("Goal cancelled")

    def create_goal_path(self):
        try:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = MAP_FRAME_ID
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = self.goal_pose
            # goal.target_pose.pose.orientation = self.robot_ori
            goal.target_pose.pose.orientation = self.goal_ori
            self.client.send_goal(goal)
            rospy.loginfo("Goal point sent to move_base")
            self.client.wait_for_result()
        except Exception as e:
            rospy.logerr("Error in sending goal point to move_base: %s" % e)

def main():
    rospy.sleep(2.0)  # Wait for the tf listener to initialize
    try:
        BagFollow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
