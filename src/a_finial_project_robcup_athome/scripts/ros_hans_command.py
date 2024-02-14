#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import time
import math

# 初始化 MediaPipe 手部解决方案
mpHands = mp.solutions.hands
hands = mpHands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils
handLmsStyle = mpDraw.DrawingSpec(color=(0, 0, 255), thickness=3)
handConStyle = mpDraw.DrawingSpec(color=(0, 255, 0), thickness=5)

# 用于计算帧率
pTime = 0
cTime = 0

def fingersUp(handLandmarks, imgWidth, imgHeight):
    if handLandmarks:
        tipIds = [4, 8, 12, 16, 20]
        fingers = []
        # Thumb
        thumb_tip_x = int(handLandmarks.landmark[tipIds[0]].x * imgWidth)
        thumb_lower_x = int(handLandmarks.landmark[tipIds[0] - 1].x * imgWidth)
        if thumb_tip_x > thumb_lower_x:  # 根据拇指的方向调整这个条件
            fingers.append(1)
        elif thumb_tip_x < thumb_lower_x:
            fingers.append(-1)
        else:
            fingers.append(0)

        
        # Fingers
        for id in range(1, 5):
            tip_x = int(handLandmarks.landmark[tipIds[id]].x * imgWidth)
            tip_y = int(handLandmarks.landmark[tipIds[id]].y * imgHeight)
            lower_x = int(handLandmarks.landmark[tipIds[id]-2].x * imgWidth)
            lower_y = int(handLandmarks.landmark[tipIds[id] - 2].y * imgHeight)
            if tip_x > lower_x:
                fingers.append(1)
            elif tip_x < lower_x:
                fingers.append(-1)
            else:
                fingers.append(0)
        return fingers
    return []

def findDistance(lmList, p1, p2, img, draw=True, r=15, t=3):
    x1, y1 = lmList[p1][1:]
    x2, y2 = lmList[p2][1:]
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

    if draw:
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), t)
        cv2.circle(img, (x1, y1), r, (255, 0, 255), cv2.FILLED)
        cv2.circle(img, (x2, y2), r, (255, 0, 255), cv2.FILLED)
        cv2.circle(img, (cx, cy), r, (0, 0, 255), cv2.FILLED)
    length = math.hypot(x2 - x1, y2 - y1)

    return length, img, [x1, y1, x2, y2, cx, cy]

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
                fingers_status = fingersUp(handLms, imgWidth, imgHeight)
                print_direction(fingers_status, cv_image)
                
                # 构建手部标记点列表
                lmList = []
                for lm in handLms.landmark:
                    lm_x, lm_y = int(lm.x * imgWidth), int(lm.y * imgHeight)
                    lmList.append([lm_x, lm_y])

                # 调用 findDistance 计算特定标记点之间的距离
                # if len(lmList) > 3:
                #     # 例如，计算食指尖（8）和拇指尖（4）之间的距离
                #     length, img, coords = findDistance(lmList, 8, 4, cv_image)
                #     # 可以在此处使用 length，例如打印或进行其他处理

    # 计算并显示帧率
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(cv_image, f"FPS : {int(fps)}", (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    # 显示处理后的图像
    cv2.imshow('Hand Tracking', cv_image)
    cv2.waitKey(1)

def print_direction(fingers_status, img):
    direction_text = ""
    if -1 in fingers_status:
        direction_text = "RIGHT"
    elif 1 in fingers_status:
        direction_text = "LEFT"
    elif fingers_status == [0,0,0,0,0]:
        direction_text = "YES"

    if direction_text:
        status_str = ', '.join(map(str, fingers_status))  # 将列表转换为字符串
        cv2.putText(img, f"{direction_text}: [{status_str}]", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    return direction_text

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
