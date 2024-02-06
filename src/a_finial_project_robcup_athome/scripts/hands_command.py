#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import time
import math

class HandDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mpDraw = mp.solutions.drawing_utils
        self.handLmsStyle = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=3)
        self.handConStyle = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=5)
        self.pTime = 0
        self.cTime = 0
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.direction_text = ""

    def fingersUp(self, handLandmarks, imgWidth, imgHeight):
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
                lower_x = int(handLandmarks.landmark[tipIds[id] - 2].x * imgWidth)
                lower_y = int(handLandmarks.landmark[tipIds[id] - 2].y * imgHeight)
                if tip_x > lower_x:
                    fingers.append(1)
                elif tip_x < lower_x:
                    fingers.append(-1)
                else:
                    fingers.append(0)
            return fingers
        return []

    def findDistance(self, lmList, p1, p2, img, draw=True, r=15, t=3):
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

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        imgRGB = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        result = self.hands.process(imgRGB)

        if result.multi_hand_landmarks:
            imgHeight, imgWidth = cv_image.shape[:2]
            for handLms in result.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(cv_image, handLms, self.mpHands.HAND_CONNECTIONS, self.handLmsStyle, self.handConStyle)
                fingers_status = self.fingersUp(handLms, imgWidth, imgHeight)
                self.print_direction(fingers_status, cv_image)

        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        cv2.putText(cv_image, f"FPS: {int(fps)}", (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.imshow('Hand Tracking', cv_image)
        cv2.waitKey(1)

    # def update_hand_direction(self, cv_image):
    #     # 处理图像并更新 hand_direction 变量
    #     imgRGB = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    #     result = self.hands.process(imgRGB)

    #     if result.multi_hand_landmarks:
    #         imgHeight, imgWidth = cv_image.shape[:2]
    #         for handLms in result.multi_hand_landmarks:
    #             self.mpDraw.draw_landmarks(cv_image, handLms, self.mpHands.HAND_CONNECTIONS, self.handLmsStyle, self.handConStyle)
    #             fingers_status = self.fingersUp(handLms, imgWidth, imgHeight)
    #             self.hand_direction = self.print_direction(fingers_status, cv_image)

    # def get_hand_direction(self):
    #     # 返回最近的手部方向
    #     return self.hand_direction

    def print_direction(self, fingers_status, img):
        self.direction_text = ""
        if -1 in fingers_status:
            self.direction_text = "RIGHT"
        elif 1 in fingers_status:
            self.direction_text = "LEFT"
        elif fingers_status == [0, 0, 0, 0, 0]:
            self.direction_text = "YES"

        if self.direction_text:
            status_str = ', '.join(map(str, fingers_status))
            cv2.putText(img, f"{self.direction_text}: [{status_str}]", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

        return self.direction_text

if __name__ == '__main__':
    rospy.init_node('hand_tracking_node', anonymous=True)
    hand_detector = HandDetector()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
