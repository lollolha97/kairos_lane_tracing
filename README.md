## AGV
---
### AGV 사용전 유의 사항
#### 1. Battery 확인
- 배터리는 90% 이상을 기준으로 정상으로 판단하겠습니다.
##### GUI를 통한 확인
1. operations.py
2. `python3 ~/AGV_UI/operations.py`
3. Battery Info 확인
#### 2. Imbeded Topic
- 모든 로직 코드(연상량이 많은 코드들)는 PC에서 실행
- 하지만 Cam, Laser Scan, IMU 등의 AGV에서 직접 발행하는 부분들은 필수적으로 AGV에서 발행
#### 0. 동시에켜기
##### `roslaunch mydata mydata_cam.launch`
##### 1. Laser Scan topic
1. operations.py
2. `python3 ~/AGV_UI/operations.py`
3.  `Laser Radar`의 `on` 버튼 클릭
4. 버튼 클릭 시 roslaunch가 실행되며 여러 데이터들 발행
5. `rostopic echo /scan`을 이용하여 scan 데이터 확인
##### 2. Camera topic
- camera topic의 경우 직접 발행
- `python3 ~/test_cam_topic_roi.py` 실행
