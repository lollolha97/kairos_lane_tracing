#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import atexit

# PID 제어 변수
I = 0
last_error = 0
twistMessage = Twist()

pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def calculatePID(error, Kp, Ki, Kd):
    global last_error, I
    P = error
    P = max(min(error, 300), -300)
    I += error
    I = max(min(I, 300), -300)
    if abs(error) < 10:
        I = 0
    D = error - last_error
    PID = int(Kp * P + Ki * I + Kd * D)
    last_error = error

    return PID

def setSpeed(speed1, speed2):
    speed2 = max(min(speed2, 50), -50)
    twistMessage.linear.x = speed1
    twistMessage.angular.z = -speed2 / 100
    pub_cmd_vel.publish(twistMessage)

def pid_control():
    # PID 제어를 위한 Subscriber 설정
    rospy.Subscriber('/deviation', Float32, deviation_callback)

def deviation_callback(data):
    error = data.data
    base_speed = 0.25
    PID = calculatePID(error, 0.2, 0.0005, 0.05)
    setSpeed(base_speed, PID)


def scan_callback(scan_data):
    safety_distance = 0.25  # 10 cm
    # 로봇 크기 고려 거리 (예: 로봇 너비의 절반)
    robot_size = 0.25  # 25 cm
    
    # 첫 10개의 거리 데이터
    first_ten_ranges = scan_data.ranges[:60]
    # 마지막 10개의 거리 데이터
    last_ten_ranges = scan_data.ranges[-60:]

    left_index = scan_data.ranges[-90:-60]
    right_index = scan_data.ranges[60:90]

    index_center = first_ten_ranges + last_ten_ranges
    index_left = right_index
    index_right = left_index

    # 각 방향별 거리 측정, 0.0을 제외
    valid_center_ranges = [r for r in index_center if scan_data.range_min < r < scan_data.range_max]
    valid_ranges_left = [r for r in index_left if r > 0.0]
    valid_ranges_right = [r for r in index_right if r > 0.0]

    distance_left = min(valid_ranges_left) if valid_ranges_left else float('inf')
    distance_center = min(valid_center_ranges) if valid_center_ranges else float('inf')
    distance_right = min(valid_ranges_right) if valid_ranges_right else float('inf')
    print("Left: {:.2f}, Center: {:.2f}, Right: {:.2f}".format(distance_left, distance_center, distance_right))
    
    # 장애물 회피 결정 로직
    if distance_center > safety_distance + robot_size:
        if distance_center < safety_distance + robot_size * 2:
            # 전방에 장애물이 있지만 회피 가능
            rospy.loginfo("Minor obstacle detected, slight adjustment...")
            adjust_path()
        else:
            # 전방에 장애물이 없으므로 전진 계속
            rospy.loginfo("Path clear, continuing forward...")
            pid_control()
    elif distance_left > distance_right and distance_left > safety_distance + robot_size:
        # 왼쪽으로 회피
        move_left()
    elif distance_right > safety_distance + robot_size:
        # 오른쪽으로 회피
        move_right()
    else:
        # 후진
        move_back()
    

def adjust_path():
    rospy.loginfo("Adjusting path to continue forward motion...")
    # 속도를 약간 감소시키고 약간 오른쪽으로 조정
    twist = Twist()
    twist.angular.z = -0.3  # 약간 오른쪽으로 회전
    pub_cmd_vel.publish(twist)

def move_stop():
    rospy.loginfo("Stop AGV")
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)

def move_left():
    rospy.loginfo("Moving left to avoid obstacle...")
    # 왼쪽으로 이동하는 명령
    twist = Twist()
    twist.linear.y = 0.3  # y축 방향으로 이동 (메카넘 휠 특성 활용)
    pub_cmd_vel.publish(twist)

def move_right():
    rospy.loginfo("Moving right to avoid obstacle...")
    # 오른쪽으로 이동하는 명령
    twist = Twist()
    twist.linear.y = -0.3  # y축 반대 방향으로 이동
    pub_cmd_vel.publish(twist)

def move_back():
    rospy.loginfo("No clear path, moving back...")
    # 후진하는 명령
    twist = Twist()
    twist.linear.x = -0.3  # 후진
    pub_cmd_vel.publish(twist)

def listener():
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
