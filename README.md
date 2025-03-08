# AGV 제어 및 시뮬레이션 프로젝트
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange?logo=ubuntu)
![ROS Noetic](https://img.shields.io/badge/ROS_Noetic-blue?logo=ros)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5.0-green?logo=opencv)
![Unity](https://img.shields.io/badge/Unity-2022.3-black?logo=unity)

[![결과영상](http://img.youtube.com/vi/pBbq8m0RX9M/0.jpg)](https://www.youtube.com/watch?v=pBbq8m0RX9M)



## 1. 목표 및 배경
### 목표    
- **Elephant Robotics MyAGV**를 이용한 **Lane Tracing**과 **Obstacle Avoidance**
- 최소 시간 트랙 완주

## 2. 시스템 아키텍처
본 프로젝트의 전체 시스템 아키텍처는 다음과 같이 구성됩니다

<img src="git_images/sys_arc.png" style="background-color: white; padding: 10px;" width="300">
<img src="git_images/dev_arc.png" style="background-color: lightgray; padding: 10px;" width="250">




## 3. 레인 트레이싱(Lane Tracing) 로직

### LaneTracing Logic
<img src="git_images/lane_tracing.png" alt="LaneTrcaing" style="max-width:600px; background-color:#ffffff; border:1px solid #ddd; padding:5px;">

### PID Controller
<img src="git_images/pid.png" alt="PID" style="max-width:600px; background-color:#ffffff; border:1px solid #ddd; padding:5px;">

#### PID On vs PID Off
| PID 적용 | PID On | PID Off |
|----------|--------|---------|
| <img src="git_images/pid_effort.png" alt="PID Effort" style="max-width:300px; background-color:#ffffff; border:1px solid #ddd; padding:5px;"> | <img src="git_images/pid_on.gif" alt="PID On" style="max-width:300px; background-color:#ffffff; border:1px solid #ddd; padding:5px;"> | <img src="git_images/pid_off.gif" alt="PID Off" style="max-width:300px; background-color:#ffffff; border:1px solid #ddd; padding:5px;"> |

### Obstacle Avoidance Logic
<img src="git_images/avoidance.png" alt="Obastacle Avoiance" style="max-width:600px; background-color:#ffffff; border:1px solid #ddd; padding:5px;">

### Traffic Light Logic
<img src="git_images/traffic_light.png" alt="Traffic Light" style="max-width:600px; background-color:#ffffff; border:1px solid #ddd; padding:5px;">

## Unity ROS#
|  적용 | PID On | PID Off|
|------|------|------|
| ![PID Effort](git_images/unity1.png) | ![dev](git_images/unity2.png) | ![dev](git_images/unity3.png) |

## 4. AGV 사용법
### AGV 사용 전 유의 사항
#### 1. 배터리 확인
- 배터리는 **90% 이상**을 정상 상태로 판단합니다.
##### GUI를 통한 확인
1. `operations.py` 실행 (MyAGV)
2. ```bash
   python3 ~/AGV_UI/operations.py
   ```
3. **Battery Info** 확인

#### 2. Imbedded Topic
- 모든 연산량이 많은 로직 코드는 **PC에서 실행**
- 하지만 **Cam, Laser Scan, IMU** 등의 **AGV 자체에서 필수적으로 발행**해야 하는 데이터들은 AGV에서 직접 처리

### 1. Laser Scan Topic (MyAGV) 실행
1. `operations.py` 실행
2. ```bash
   python3 ~/AGV_UI/operations.py
   ```
3. **Laser Radar**의 `On` 버튼 클릭
4. 버튼 클릭 시 `roslaunch`가 실행되며 여러 센서 데이터가 발행됨
5. Laser Scan 데이터 확인
   ```bash
   rostopic echo /scan
   ```

### 2. Camera Topic (MyAGV) 실행
- Camera topic은 직접 발행해야 함
- 실행 명령어:
   ```bash
   python3 ~/test_cam_topic_roi.py
   ```

## 5. 설치 및 실행 방법

### 환경 설정 및 실행
1. **ROS 환경 설정**
   ```bash
   source /opt/ros/noetic/setup.bash
   ```
2. **패키지 빌드**
   ```bash
   cd src
   catkin_make
   ```
3. **AGV 제어 시스템 실행**
   ```bash
   roslaunch agv_control start.launch
   ```


## 6. 기술 스택
- **프로그래밍 언어**: Python, C++
- **프레임워크 및 라이브러리**: ROS Noetic, OpenCV, Unity ML-Agents
- **시뮬레이션 환경**: Unity 3D
- **하드웨어 연동**: AGV 센서 및 모터 제어 API

## 7. 참고 자료
- [ROS 공식 문서](https://www.ros.org/)
- [OpenCV 공식 문서](https://opencv.org/)
- [Unity ML-Agents](https://github.com/Unity-Technologies/ml-agents)
- [Unity ROS#](https://github.com/siemens/ros-sharp)

