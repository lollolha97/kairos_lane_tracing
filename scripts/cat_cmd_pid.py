#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

id_node = "lane"

I = 0
last_error = 0
twistMessage = Twist()

pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def calculatePID(error, Kp, Ki, Kd):
    global last_error, I

    P = error
    P = max(min(P, 300), -300)  # P 값을 -300에서 300 사이로 제한

    I = I + error
    I = max(min(I, 300), -300)  # I 값을 -300에서 300 사이로 제한

    if abs(error) < 10:
        I = 0  # 작은 오차에서는 I 항을 0으로 리셋

    D = error - last_error
    PID = int(Kp * P + Ki * I + Kd * D)
    last_error = error

    return PID

def setSpeed(speed1, speed2):
    # 회전 속도 제한을 적용하여 로봇의 움직임을 더 안정화
    speed2 = max(min(speed2, 100), -100)  # 회전 속도를 -50에서 50 사이로 제한
    twistMessage.linear.x = speed1
    twistMessage.angular.z = -speed2 / 100  # 스케일링 인자 조정
    pub_cmd_vel.publish(twistMessage)

def callback(data):
    error = data.data
    # base_speed = 0.25  # 기본 속도 설정
    base_speed = 0.4  # 기본 속도 설정
    # PID = calculatePID(error, 0.2, 0.0005, 0.05)  # PID 계수를 조정하여 더 부드러운 반응을 얻음
    PID = calculatePID(error, 0.3, 0.0005, 0.05)  # PID 계수를 조정하여 더 부드러운 반응을 얻음
    print("error: ", error)
    setSpeed(base_speed, PID)

def lane_controller():
    rospy.init_node('lane_controller', anonymous=True)
    rospy.Subscriber('/deviation', Float32, callback)
    # show error
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    lane_controller()