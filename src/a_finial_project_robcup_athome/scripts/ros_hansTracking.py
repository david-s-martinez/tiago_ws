#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import time

# 初始化 MediaPipe 手部解决方案
mpHands = mp.solutions.hands
hands = mpHands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils
handLmsStyle = mpDraw.DrawingSpec(color=(0, 0, 255), thickness=3)
handConStyle = mpDraw.DrawingSpec(color=(0, 255, 0), thickness=5)

# 用于计算帧率
pTime = 0
cTime = 0

def image_callback(ros_image):
    global pTime, cTime
    try:
        # 将 ROS 图像转换为 OpenCV 图像
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # 进行手部识别
    imgRGB = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    result = hands.process(imgRGB)

    if result.multi_hand_landmarks:
        imgHeight, imgWidth = cv_image.shape[:2]
        for handLms in result.multi_hand_landmarks:
            mpDraw.draw_landmarks(cv_image, handLms, mpHands.HAND_CONNECTIONS, handLmsStyle, handConStyle)
            for i, lm in enumerate(handLms.landmark):
                xPos = int(lm.x * imgWidth)
                yPos = int(lm.y * imgHeight)
                cv2.putText(cv_image, str(i), (xPos-25, yPos+5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)

    # 计算并显示帧率
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(cv_image, f"FPS : {int(fps)}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    # 显示处理后的图像
    cv2.imshow('Hand Tracking', cv_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('hand_tracking_node', anonymous=True)
    bridge = CvBridge()
    # 订阅 TIAGO 摄像头的 ROS 话题
    image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
