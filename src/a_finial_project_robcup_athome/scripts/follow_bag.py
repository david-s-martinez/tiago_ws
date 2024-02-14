#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib
import numpy as np

class LookToBag:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("look_to_bag")
        rospy.loginfo("Starting look_to_point application ...")

        # Global variables
        self.window_name = "head"
        self.camera_frame = "/xtion_rgb_optical_frame"
        self.image_topic = "/xtion/rgb/image_raw"
        self.camera_info_topic = "/xtion/rgb/camera_info"
        self.centroid_goal_topic = '/pick_centroid'
        #self.centroid_goal_topic = '/goal_centroid'
        self.camera_intrinsics = None
        self.latest_image_stamp = None
        self.point_head_client = None
        self.last_centroid = None
        self.rect_width = 200  # Adjust according to your requirements
        self.rect_height = 150  # Adjust according to your requirements
        self.img_dims = (480,640,3)
        self.image_center_x = int(self.img_dims[1]/2)
        self.image_center_y = int(self.img_dims[0]/2)
        # Create a point head action client
        self.create_point_head_client()

        # Subscribe to the image topic
        rospy.loginfo("Subscribing to " + self.image_topic + " ...")
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber('/bag_status', String, self.finish_callback)
        # Subscribe to the topic where the detections are being published
        self.centroid_sub = rospy.Subscriber(self.centroid_goal_topic, String, self.centroid_callback, buff_size=1)

    def finish_callback(self, msg):
        self.finish = msg.data
        if self.finish == "Success":
            rospy.signal_shutdown("Complete")


    def centroid_callback(self, centroid_msg):
        # rospy.sleep(0.5)
        # Decode the centroid string into a list of floating point numbers
        new_centroid = eval(centroid_msg.data)
        #print(type(new_centroid))
        #print(self.last_centroid, new_centroid)
        self.last_centroid = new_centroid
        if self.last_centroid is not None:
            # Call the function to make the camera point to the centroid
            self.point_to_centroid(self.last_centroid)


    # ROS callback for every new image received
    def image_callback(self, img_msg):
        self.latest_image_stamp = img_msg.header.stamp

        cv_img = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")
        self.img_dims = cv_img.shape
        if self.last_centroid != None:
            cv2.circle(cv_img, tuple(self.last_centroid), radius=10, color=(0, 0, 255), thickness=-1)
        rect_top_left = (self.image_center_x - self.rect_width // 2, self.image_center_y - self.rect_height // 2)
        rect_bottom_right = (self.image_center_x + self.rect_width // 2, self.image_center_y + self.rect_height // 2)
        cv2.rectangle(cv_img, rect_top_left, rect_bottom_right, (0, 255, 0), thickness=2)
        cv2.imshow(self.window_name, cv_img)
        cv2.setMouseCallback(self.window_name, self.on_mouse)
        key = cv2.waitKey(1)
        if key == 27:
            rospy.signal_shutdown('User requested shutdown')
        
        # Function to make the camera point to the centroid
    def point_to_centroid(self, centroid):
        # Define the dimensions of the rectangle (width and height)
        
        # Define the center of the image frame
        image_center_x = self.img_dims[1]/2
        image_center_y = self.img_dims[0]/2

        # Compute the boundaries of the rectangle
        rect_left = image_center_x - self.rect_width / 2
        rect_right = image_center_x + self.rect_width / 2
        rect_top = image_center_y - self.rect_height / 2
        rect_bottom = image_center_y + self.rect_height / 2

        # Check if the centroid falls within the rectangle
        if not(rect_left <= centroid[0] <= rect_right and rect_top <= centroid[1] <= rect_bottom):
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = rospy.Time.now()

            # Compute normalized coordinates of the centroid pixel
            x = (centroid[0] - self.camera_intrinsics[0, 2]) / self.camera_intrinsics[0, 0]
            y = (centroid[1] - self.camera_intrinsics[1, 2]) / self.camera_intrinsics[1, 1]
            Z = 1.0  # Define an arbitrary distance

            point_stamped.point.x = x * Z
            point_stamped.point.y = y * Z
            point_stamped.point.z = Z

            goal = PointHeadGoal()
            goal.pointing_frame = self.camera_frame
            goal.pointing_axis.x = 0.0
            goal.pointing_axis.y = 0.0
            goal.pointing_axis.z = 1.0
            goal.min_duration = rospy.Duration(0.5)
            goal.max_velocity = 0.07
            goal.target = point_stamped

            self.point_head_client.send_goal(goal)
        else:
            rospy.loginfo("Centroid is outside the predefined rectangle. Skipping goal.")

    # OpenCV callback function for mouse events on a window
    def on_mouse(self, event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        rospy.loginfo(f"Pixel selected ({u}, {v}) Making TIAGo look to that direction")
        self.point_to_centroid([u,v])

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
    look_to_point = LookToBag()
    look_to_point.run()
