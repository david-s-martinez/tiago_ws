#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib
import numpy as np

class LookToPoint:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("look_to_point")
        rospy.loginfo("Starting look_to_point application ...")

        # Global variables
        self.window_name = "head"
        self.camera_frame = "/xtion_rgb_optical_frame"
        self.image_topic = "/xtion/rgb/image_raw"
        self.camera_info_topic = "/xtion/rgb/camera_info"
        self.camera_intrinsics = None
        self.latest_image_stamp = None
        self.point_head_client = None

        # Create a point head action client
        self.create_point_head_client()

        # Subscribe to the image topic
        rospy.loginfo("Subscribing to " + self.image_topic + " ...")
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

    # ROS callback for every new image received
    def image_callback(self, img_msg):
        self.latest_image_stamp = img_msg.header.stamp

        cv_img = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imshow(self.window_name, cv_img)
        cv2.setMouseCallback(self.window_name, self.on_mouse)
        cv2.waitKey(1)

    # OpenCV callback function for mouse events on a window
    def on_mouse(self, event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        rospy.loginfo(f"Pixel selected ({u}, {v}) Making TIAGo look to that direction")

        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.header.stamp = self.latest_image_stamp

        # Compute normalized coordinates of the selected pixel
        x = (u - self.camera_intrinsics[0, 2]) / self.camera_intrinsics[0, 0]
        y = (v - self.camera_intrinsics[1, 2]) / self.camera_intrinsics[1, 1]
        Z = 1.0  # Define an arbitrary distance
        point_stamped.point.x = x * Z
        point_stamped.point.y = y * Z
        point_stamped.point.z = Z

        # Build the action goal
        goal = PointHeadGoal()
        goal.pointing_frame = self.camera_frame
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(1.0)
        goal.max_velocity = 0.25
        goal.target = point_stamped

        self.point_head_client.send_goal(goal)
        rospy.sleep(0.5)

    # Create a ROS action client to move TIAGo's head
    def create_point_head_client(self):
        rospy.loginfo("Creating action client to head controller ...")
        self.point_head_client = actionlib.SimpleActionClient("/head_controller/point_head_action", PointHeadAction)
        self.point_head_client.wait_for_server(rospy.Duration(25))

    # Run the node
    def run(self):
        # Get the camera intrinsic parameters from the appropriate ROS topic
        rospy.loginfo("Waiting for camera intrinsics ... ")
        msg = rospy.wait_for_message(self.camera_info_topic, CameraInfo)
        if msg is not None:
            fx = msg.K[0]
            fy = msg.K[4]
            cx = msg.K[2]
            cy = msg.K[5]
            self.camera_intrinsics = np.array([[fx, 0, cx],
                                               [0, fy, cy],
                                               [0, 0, 1]])

        # Enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
        rospy.spin()

        # Destroy the OpenCV window
        cv2.destroyWindow(self.window_name)

if __name__ == '__main__':
    look_to_point = LookToPoint()
    look_to_point.run()
