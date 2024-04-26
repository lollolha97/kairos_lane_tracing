#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower', anonymous=True)

        # 이미지 구독 설정
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)
        self.bridge = CvBridge()

        # 정지 여부 퍼블리셔 설정
        self.stop_pub = rospy.Publisher('/isStop', Bool, queue_size=1)

    def process_image(self, img):
        # HSV 색공간으로 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 빨간색 범위 설정
        lower_red1 = np.array([0, 100, 100])  # 낮은 색조의 빨간색 범위 시작
        upper_red1 = np.array([10, 255, 255])  # 낮은 색조의 빨간색 범위 끝
        lower_red2 = np.array([160, 100, 100])  # 높은 색조의 빨간색 범위 시작
        upper_red2 = np.array([179, 255, 255])  # 높은 색조의 빨간색 범위 끝

        # 빨간색 마스크 생성
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # Contour 추출
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            return area > 2000, mask, img  # 2000 넘으면 True, 아니면 False 반환
        return False, mask, img

    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            stop, mask, processed_image = self.process_image(cv_image)
            self.stop_pub.publish(stop)  # Bool 값으로 발행
            if stop:
                print("Stopping due to large red contour detected")
            else:
                print("No large red contour detected, continuing...")

            # 이미지 시각화
            cv2.imshow("Processed Image", processed_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        lf = LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
