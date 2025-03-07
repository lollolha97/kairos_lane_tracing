![Ubuntu 20.04](https://upload.wikimedia.org/wikipedia/commons/a/af/Ubuntu-logo-small.svg) ![ROS Noetic](https://raw.githubusercontent.com/ros-infrastructure/roswiki/master/macro/ros.svg) ![Python](https://upload.wikimedia.org/wikipedia/commons/c/c3/Python-logo-notext.svg) ![OpenCV](https://upload.wikimedia.org/wikipedia/commons/3/32/OpenCV_Logo_with_text_svg_version.svg)

# AGV 제어 및 시뮬레이션 프로젝트

## 1. 목표 및 배경
### 목표
본 프로젝트는 **ROS Noetic**을 활용하여 **AGV(Automated Guided Vehicle) 제어 시스템**을 개발하고, **Unity 기반의 시뮬레이션 환경**을 구축하는 것을 목표로 합니다. AGV의 주행 경로를 최적화하고 센서 데이터를 효율적으로 처리하여 **산업 자동화 환경에서 활용 가능한 자율주행 시스템**을 구현하는 것이 핵심 목표입니다.

### 배경
AGV는 물류 및 제조업에서 중요한 자동화 요소로 자리 잡고 있습니다. 기존의 AGV 시스템은 정해진 경로를 따라 이동하는 방식이 일반적이지만, 최근에는 **SLAM(Simultaneous Localization and Mapping)**, **컴퓨터 비전**, **레이 트레이싱(ray tracing) 로직** 등을 활용하여 보다 정밀한 경로 탐색과 장애물 회피 기능을 수행하는 방향으로 발전하고 있습니다. 본 프로젝트에서는 ROS 및 OpenCV를 활용하여 **AGV의 인공지능 기반 경로 탐색 및 시뮬레이션 기능을 개선**하는 것을 목표로 합니다.

## 2. 시스템 아키텍처
본 프로젝트의 전체 시스템 아키텍처는 다음과 같이 구성됩니다:


## 3. 레인 트레이싱(Ray Tracing) 로직


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

### 0. AGV 및 센서 동시에 실행 (MyAGV)
```bash
roslaunch mydata mydata_cam.launch
```

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
### 사전 요구 사항
- Ubuntu 20.04
- ROS Noetic
- Python 3.x
- OpenCV
- Unity 3D 2022.03

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

