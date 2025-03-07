import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
import threading


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(Image, '/rgb', self.image_callback, 10)
        self.bridge = CvBridge()
        self.latest_image = None

    def image_callback(self, msg):
        """ ROS 2에서 /rgb 이미지 데이터를 수신하고 OpenCV 형식으로 변환 """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            print("📸 이미지 수신 완료!")  # 터미널에서 확인
        except Exception as e:
            print(f"❌ 이미지 변환 오류: {e}")


class HSVAdjuster:
    def __init__(self, root, node):
        self.node = node
        self.root = root
        self.root.title("ROS2 HSV Adjuster")

        # 초기 HSV 값
        self.lower_hue = tk.IntVar(value=28)
        self.lower_saturation = tk.IntVar(value=125)
        self.lower_value = tk.IntVar(value=98)
        self.upper_hue = tk.IntVar(value=42)
        self.upper_saturation = tk.IntVar(value=255)
        self.upper_value = tk.IntVar(value=239)

        # 슬라이더 UI
        self.create_sliders()

        # 현재 HSV 값 표시
        self.lower_label = ttk.Label(root, text=f"Lower HSV: {self.get_lower_values()}")
        self.lower_label.pack()
        self.upper_label = ttk.Label(root, text=f"Upper HSV: {self.get_upper_values()}")
        self.upper_label.pack()

        # 이미지 표시용 Canvas
        self.canvas = tk.Canvas(root, width=640, height=480)
        self.canvas.pack()

        # 실시간 이미지 업데이트
        self.update_image()

    def create_sliders(self):
        """ HSV 조절용 슬라이더 UI 생성 """
        frame = ttk.Frame(self.root)
        frame.pack()

        sliders = [
            ("Lower Hue", self.lower_hue, 0, 180),
            ("Lower Saturation", self.lower_saturation, 0, 255),
            ("Lower Value", self.lower_value, 0, 255),
            ("Upper Hue", self.upper_hue, 0, 180),
            ("Upper Saturation", self.upper_saturation, 0, 255),
            ("Upper Value", self.upper_value, 0, 255),
        ]

        for label, var, min_val, max_val in sliders:
            ttk.Label(frame, text=label).pack()
            ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL, variable=var, command=self.update_hsv_labels).pack()

    def get_lower_values(self):
        return f"[{self.lower_hue.get()}, {self.lower_saturation.get()}, {self.lower_value.get()}]"

    def get_upper_values(self):
        return f"[{self.upper_hue.get()}, {self.upper_saturation.get()}, {self.upper_value.get()}]"

    def update_hsv_labels(self, event=None):
        """ HSV 값을 UI 및 터미널에 실시간 출력 """
        lower_values = self.get_lower_values()
        upper_values = self.get_upper_values()

        self.lower_label.config(text=f"Lower HSV: {lower_values}")
        self.upper_label.config(text=f"Upper HSV: {upper_values}")

        print(f"🎨 현재 HSV 값 - Lower: {lower_values}, Upper: {upper_values}")

    def update_image(self):
        """ 실시간으로 ROS 2 이미지를 받아서 HSV 변환 후 표시 """
        if self.node.latest_image is not None:
            img = self.node.latest_image.copy()

            # HSV 변환
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower = np.array([self.lower_hue.get(), self.lower_saturation.get(), self.lower_value.get()])
            upper = np.array([self.upper_hue.get(), self.upper_saturation.get(), self.upper_value.get()])
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(img, img, mask=mask)

            # OpenCV → PIL 변환 후 tkinter에 표시
            img_rgb = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
            img_pil = PILImage.fromarray(img_rgb)
            img_tk = ImageTk.PhotoImage(img_pil)

            self.canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
            self.canvas.img = img_tk  # 가비지 컬렉션 방지

        # 30ms 간격으로 업데이트
        self.root.after(30, self.update_image)


def main():
    rclpy.init()
    node = ImageProcessor()

    root = tk.Tk()
    app = HSVAdjuster(root, node)

    # ROS 2 노드를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
