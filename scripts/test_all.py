#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

# PID 제어 변수
I = 0
last_error = 0
twistMessage = Twist()
is_stop_signaled = False  # 정지 신호 상태

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
    if is_stop_signaled:  # 정지 신호가 활성화된 경우 속도 조절을 하지 않음
        return
    speed2 = max(min(speed2, 50), -50)
    twistMessage.linear.x = speed1
    twistMessage.angular.z = -speed2 / 100
    pub_cmd_vel.publish(twistMessage)

def traffic_light_callback(data):
    global is_stop_signaled
    if data.data:
        is_stop_signaled = True
        move_stop()
    else:
        is_stop_signaled = False

def deviation_callback(data):
    if is_stop_signaled:  # 정지 신호가 활성화된 경우 PID 제어를 하지 않음
        return
    error = data.data
    base_speed = 0.25
    PID = calculatePID(error, 0.2, 0.0005, 0.05)
    setSpeed(base_speed, PID)

def scan_callback(scan_data):
    safety_distance = 0.25  # 30 cm
    robot_size = 0.25  # 30 cm

    first_ten_ranges = scan_data.ranges[:60]
    last_ten_ranges = scan_data.ranges[-60:]
    left_index = scan_data.ranges[-90:-60]
    right_index = scan_data.ranges[60:90]

    index_center = first_ten_ranges + last_ten_ranges
    index_left = right_index
    index_right = left_index

    valid_center_ranges = [r for r in index_center if scan_data.range_min < r < scan_data.range_max]
    valid_left_ranges = [r for r in index_left if r > 0.0]
    valid_right_ranges = [r for r in index_right if r > 0.0]

    distance_left = min(valid_left_ranges) if valid_left_ranges else float('inf')
    distance_center = min(valid_center_ranges) if valid_center_ranges else float('inf')
    distance_right = min(valid_right_ranges) if valid_right_ranges else float('inf')
    print("Left: {:.2f}, Center: {:.2f}, Right: {:.2f}".format(distance_left, distance_center, distance_right))

    if distance_center > safety_distance + robot_size:
        if distance_center < safety_distance + robot_size * 2:
            rospy.loginfo("Minor obstacle detected, slight adjustment...")
            adjust_path()
        else:
            rospy.loginfo("Path clear, continuing forward...")
            deviation_callback(Float32(data=0))  # PID 제어로 전진
    elif distance_left > distance_right and distance_left > safety_distance + robot_size:
        move_left()
    elif distance_right > safety_distance + robot_size:
        move_right()
    else:
        move_back()

def adjust_path():
    twist = Twist()
    twist.angular.z = -0.1
    pub_cmd_vel.publish(twist)

def move_stop():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)

def move_left():
    twist = Twist()
    twist.linear.y = 0.3
    pub_cmd_vel.publish(twist)

def move_right():
    twist = Twist()
    twist.linear.y = -0.3
    pub_cmd_vel.publish(twist)

def move_back():
    twist = Twist()
    twist.linear.x = -0.25
    pub_cmd_vel.publish(twist)

def listener():
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/isStop", Bool, traffic_light_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
