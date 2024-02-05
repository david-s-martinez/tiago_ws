#!/usr/bin/env python

import rospy
import actionlib
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
import numpy as np


class LookToPoint:
    def __init__(self):
        self.node_name = "look_to_point"
        rospy.init_node(self.node_name)

        self.window_name = "Inside of TIAGo's head"
        self.camera_frame = "/xtion_rgb_optical_frame"
        self.image_topic = "/xtion/rgb/image_raw"
        self.camera_info_topic = "/xtion/rgb/camera_info"

        self.camera_intrinsics = None
        self.latest_image_stamp = None
        self.bridge = CvBridge()

        self.point_head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        rospy.loginfo("Waiting for head controller action server...")
        # self.point_head_client.wait_for_server()

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

    def camera_info_callback(self, msg):
        self.camera_intrinsics = np.zeros((3, 3), dtype=np.float64)
        self.camera_intrinsics[0, 0] = msg.K[0] # fx
        self.camera_intrinsics[1, 1] = msg.K[4] # fy
        self.camera_intrinsics[0, 2] = msg.K[2] # cx
        self.camera_intrinsics[1, 2] = msg.K[5] # cy
        self.camera_intrinsics[2, 2] = 1


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(3)
        self.latest_image_stamp = data.header.stamp

    def on_mouse(self, event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        rospy.loginfo("Pixel selected ({}, {}) Making TIAGo look to that direction".format(u, v))

        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.header.stamp = self.latest_image_stamp

        x = (u - self.camera_intrinsics[0, 2]) / self.camera_intrinsics[0, 0]
        y = (v - self.camera_intrinsics[1, 2]) / self.camera_intrinsics[1, 1]
        Z = 1.0 # Define an arbitrary distance
        point_stamped.point.x = x * Z
        point_stamped.point.y = y * Z
        point_stamped.point.z = Z

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

if __name__ == '__main__':
    try:
        look_to_point = LookToPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
