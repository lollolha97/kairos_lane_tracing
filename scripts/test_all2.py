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
    P = max(min(P, 300), -300)
    I += error
    I = max(min(I, 300), -300)
    if abs(error) < 10:
        I = 0
    D = error - last_error
    PID = int(Kp * P + Ki * I + Kd * D)
    last_error = error
    return PID

def setSpeed(speed1, speed2):
    global is_stop_signaled
    if is_stop_signaled:
        move_stop()
        return
    speed2 = max(min(speed2, 50), -50)
    twistMessage.linear.x = speed1
    twistMessage.angular.z = -speed2 / 100
    pub_cmd_vel.publish(twistMessage)

def stop_signal_callback(data):
    global is_stop_signaled
    is_stop_signaled = data.data
    if is_stop_signaled:
        move_stop()

def scan_callback(scan_data):
    global is_stop_signaled

    safety_distance = 0.25  # 25 cm
    robot_size = 0.25  # 25 cm

    # 각 방향별 거리 측정, 0.0을 제외
    center_ranges = scan_data.ranges[60:120] + scan_data.ranges[-120:-60]
    left_ranges = scan_data.ranges[-180:-120]
    right_ranges = scan_data.ranges[120:180]

    valid_center = [r for r in center_ranges if scan_data.range_min < r < scan_data.range_max]
    valid_left = [r for r in left_ranges if r > 0.0]
    valid_right = [r for r in right_ranges if r > 0.0]

    distance_left = min(valid_left) if valid_left else float('inf')
    distance_center = min(valid_center) if valid_center else float('inf')
    distance_right = min(valid_right) if valid_right else float('inf')
    
    if is_stop_signaled:
        move_stop()
    else:
        # 장애물 회피 결정 로직
        if distance_center > safety_distance + robot_size:
            rospy.loginfo("Path clear, continuing forward...")
            # PID 제어가 필요한 경우 여기서 활성화
        elif distance_left > distance_right and distance_left > safety_distance + robot_size:
            move_left()
        elif distance_right > safety_distance + robot_size:
            move_right()
        else:
            move_back()

def move_stop():
    rospy.loginfo("Stopping due to stop signal")
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)

def move_left():
    rospy.loginfo("Moving left to avoid obstacle")
    twist = Twist()
    twist.linear.y = 0.3
    pub_cmd_vel.publish(twist)

def move_right():
    rospy.loginfo("Moving right to avoid obstacle")
    twist = Twist()
    twist.linear.y = -0.3
    pub_cmd_vel.publish(twist)

def move_back():
    rospy.loginfo("No clear path, moving back")
    twist = Twist()
    twist.linear.x = -0.25
    pub_cmd_vel.publish(twist)

def listener():
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/isStop", Bool, stop_signal_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
