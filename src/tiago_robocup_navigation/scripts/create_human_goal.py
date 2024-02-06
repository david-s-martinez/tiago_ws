#!/usr/bin/env python3
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, PointStamped
import math

# Topic names and frame ids as constants for easy modification
NODE_NAME = "create_human_goal" # Name of the ROS node
GOAL_SUBSCRIBE = "/goal_point" # Topic to subscribe to for the goal point from the camera
GOAL_PUBLISH = "/human_goal" # Topic to publish the goal point to move_base
CAMERA_FRAME_ID = "xtion_rgb_optical_frame" # Frame id of the camera
MAP_FRAME_ID = "map"

class GetTargetPoint:
    def __init__(self):
        # Initialize ROS nodes
        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.Subscriber(GOAL_SUBSCRIBE, PointStamped, self.point_callback)
        self.listener = tf.TransformListener()
        self.goal_pub = rospy.Publisher(GOAL_PUBLISH, PoseStamped, queue_size=2)
        rospy.loginfo("f{NODE_NAME} initialized")

    def point_callback(self, msg):
        camera_point = geometry_msgs.msg.PointStamped()
        camera_point.header.frame_id = CAMERA_FRAME_ID
        camera_point.point = msg.point

        if camera_point.point.x is None or camera_point.point.y is None or camera_point.point.z is None:
            rospy.loginfo("No point received from the camera")
            return

        try:
            # Wait until the transform is available
            self.listener.waitForTransform(MAP_FRAME_ID, CAMERA_FRAME_ID, rospy.Time(0), rospy.Duration(4.0))
            # Perform transformation
            transformed_point = self.listener.transformPoint(MAP_FRAME_ID, camera_point)
            
            # Prepare the goal point
            goal_point = geometry_msgs.msg.PoseStamped()
            goal_point.header.frame_id = MAP_FRAME_ID
            goal_point.header.stamp = rospy.Time.now()
            goal_point.pose.position.x = transformed_point.point.x
            goal_point.pose.position.y = transformed_point.point.y - 0.5 * math.sin(math.atan2(transformed_point.point.y, transformed_point.point.x))
            goal_point.pose.position.z = transformed_point.point.z
            
            # Calculate orientation
            goal_point.pose.orientation.x = 0.0
            goal_point.pose.orientation.y = 0.0
            goal_point.pose.orientation.z = 0.0
            goal_point.pose.orientation.w = 1.0
            
            # rospy.loginfo("Publishing Goal point: x=%f, y=%f, z=%f" % (goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z))
            self.goal_pub.publish(goal_point)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Transform Error: %s" % e)

def main():
    rospy.sleep(2.0)  # Wait for tf listener to initialize
    GetTargetPoint()
    rospy.spin()

if __name__ == '__main__':
    main()
