import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import atexit
import time


class PIDController:
    """ PID 제어 클래스 """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # 비례 게인
        self.Ki = Ki  # 적분 게인
        self.Kd = Kd  # 미분 게인
        self.prev_error = 0.0  # 이전 오차값
        self.integral = 0.0  # 누적 오차값
        self.prev_time = time.time()

    def compute(self, error):
        """ PID 연산 수행 """
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt == 0:
            return 0  # 나누기 0 방지

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 업데이트
        self.prev_error = error
        self.prev_time = current_time

        return output


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        # 이미지 구독 설정
        self.subscriber = self.create_subscription(Image, "/rgb", self.callback, 1)
        self.bridge = CvBridge()

        # cmd_vel 퍼블리셔 설정
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # PID 컨트롤러 생성 (Kp, Ki, Kd 설정)
        # 1. P 값 맞춤
        # 2. P 값의 영향을 줄이기 위해 D 값 증가 (P값 대비 D값이 작으면 오버슈팅 발생)
        # 3. I 값 증가 (P, D 값으로는 해결할 수 없는 오차를 보정)
        self.pid_controller = PIDController(Kp=4.0, Ki=0.0, Kd=0.0)

        # 프로그램 종료 시 호출될 함수 등록
        atexit.register(self.stop_agv)

    def process_image(self, img):
        """ 이미지를 HSV 변환 후 차선 검출 """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0, 30, 172])
        upper_yellow = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # 무게중심 좌표 계산
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy), mask
        return None, mask

    def callback(self, data):
        """ 이미지 콜백 함수 - 차선 유지 제어 """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            result = self.process_image(cv_image)

            twist = Twist()
            if result and result[0] is not None:
                cx, cy = result[0]
                img_center = cv_image.shape[1] / 2  # 이미지 중앙
                deviation = cx - img_center  # 차선 중앙에서의 오차

                # PID 제어를 이용한 각속도 조절
                pid_output = self.pid_controller.compute(-deviation / 150)

                twist.linear.x = 1.8  # 기본 전진 속도
                twist.angular.z = pid_output  # PID 보정된 각속도 적용
            else:
                twist.angular.z = 0.15  # 차선을 찾지 못하면 천천히 우회전

            self.cmd_vel_pub.publish(twist)

        except CvBridgeError as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")

    def stop_agv(self):
        """ AGV 정지 """
        print("Stopping AGV...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    lane_follower = LaneFollower()

    rclpy.spin(lane_follower)

    lane_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
