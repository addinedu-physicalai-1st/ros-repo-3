# Phase 4.9: GUI Display and Sensor Hardware Error Handling Fix Report

## 1. 개요
본 문서는 Pinky PC GUI(`pinky_station`)에서의 화면 미출력 문제, 조작 편의성 개선, 그리고 로봇 코어(`pinky_core`) 센서 하드웨어(BNO055 IMU)의 연결 실패 시 발생하던 시스템 멈춤 현상에 대한 수정 및 개선 내역을 요약합니다.

## 2. 주요 수정 내역

### 2.1. GUI 카메라 영상 수신 및 출력 구현
*   **문제**: PC GUI 화면에서 카메라 피드가 나타나지 않음. 로봇(`pinky_core`) 측에 카메라 드라이버 인터페이스만 있고 실제 이미지를 캡처하여 TCP로 전송하는 구현체가 부재함.
*   **수정**:
    *   `pinky_core/include/pinky_core/hal/opencv_camera.h` 및 `pinky_core/src/hal/opencv_camera.cpp` 신규 작성.
    *   OpenCV(`cv::VideoCapture`)를 활용하되, 라즈베리파이의 `libcamera` 환경을 고려하여 GStreamer 파이프라인(`libcamerasrc`)을 최우선으로 시도하고, 실패 시 V4L2(`/dev/video0`)로 폴백하도록 구현.
    *   기존 파이썬 환경(`pinkylib`)과 하드웨어 방향을 맞추기 위해 영상을 180도 회전(`cv::ROTATE_180`) 적용 후 JPEG 압축 전송.
    *   `RobotApp::Init()`에서 `OpencvCamera` 객체를 정상적으로 할당(`make_unique`)하도록 초기화 로직 보완.

### 2.2. GUI 라이다(LiDAR) 2D 스캔 그래프 출력 에러 수정
*   **문제**: 
    1. PyQtGraph의 ScatterPlotItem 인자 전달 타입 문제.
    2. 로봇 코어(`pinky_core`)의 `LidarProcessor`가 24등분 섹터의 최솟값을 풀링(Min-Pooling)할 때, 측정 불가(0.0) 값을 그대로 최솟값으로 취해버리는 논리적 오류가 발생. 이로 인해 GUI에서 0.0인 섹터를 무시하게 되어 Lidar 그래프가 전체적으로 비어보이는 현상 발생.
*   **수정**:
    *   **GUI 측**: 명시적 키워드 인자(`self.scatter.setData(x=np.array(x_pts), y=np.array(y_pts))`)와 Numpy 배열 변환을 통해 안정적인 렌더링 지원.
    *   **로봇 코어 측**: `pinky_core/src/core/lidar_processor.cpp`의 섹터 Min-Pooling 로직에서 유효하지 않은 측정값(`0.05f` 이하)을 무시하도록 조건문(`if (cleaned[j] > 0.05f)`) 추가.

### 2.3. LCD 감정 표현(GIF) 로딩 시 OpenCV 충돌 에러 수정
*   **문제**: 로봇 실행 시 `OpenCV | GStreamer warning: Error opening bin: no element "emotion"` 및 `CAP_IMAGES: can't find starting number` 에러 발생과 함께 LCD 출력이 무시됨. 이는 OpenCV의 `VideoCapture`가 기본적으로 GIF 애니메이션 파싱을 지원하지 않아 이미지 시퀀스로 오해하고 발생하는 오류임.
*   **수정**:
    *   `pinky_core/src/core/emotion_renderer.cpp` 파일의 `LoadAnimatedEmotion` 함수에서 OpenCV(`cv::VideoCapture`) 기반 GIF 로딩 코드를 완전히 제거.
    *   기존 프로젝트에 포함되어 있던 C 헤더 라이브러리인 `stb_image.h`(`stbi_load_gif_from_memory`)를 단독으로 사용하여 GIF 애니메이션 프레임들을 안정적으로 로드하도록 일원화.

### 2.4. 맵 위젯(MapWidget) 뷰포트 좌표계 기준 변경
*   **문제**: 로봇이 고정된 상태에서 오도메트리 이동 시 맵의 월드 좌표계 자체가 로봇을 따라다니는 형태로 렌더링되어 사용자가 로봇의 실제 이동 궤적을 시각적으로 파악하기 어려움.
*   **수정**:
    *   `pinky_station/gui/widgets/map_widget.py` 파일의 변환(Transform) 계산 로직 수정.
    *   카메라 뷰포트의 중심이 로봇의 현재 위치(`self.robot_x`, `self.robot_y`)가 아닌 **월드 원점(0, 0)**을 향하도록 변경.
    *   디버깅 편의성을 위해 월드 원점을 표시하는 X/Y(빨강/초록) 고정 축은 유지하고, 로봇이 이 원점을 기준으로 오도메트리 이동량만큼 캔버스 상에서 움직이도록(Absolute coordinate rendering) 개선.

### 2.5. 원격 조작(Teleop) 속도 조절 분리 및 WASD 제어 추가
*   **문제**: 기존 Teleop 제어 위젯에 속도 조절 슬라이더가 1개뿐이어서 전진(Linear)과 회전(Angular) 속도를 독립적으로 제어할 수 없었으며 마우스 클릭으로만 제어가 가능했음.
*   **수정**:
    *   `pinky_station/gui/widgets/teleop_widget.py` 파일 수정.
    *   병진 속도 조절용 `L-Speed` 슬라이더와 회전 속도 조절용 `A-Speed` 슬라이더를 분리.
    *   회전 속도(Angular Velocity)는 0.1 ~ 3.0 rad/s 범위로 설정 가능하도록 구현.
    *   `keyPressEvent`와 `keyReleaseEvent`를 추가하여 WASD 키보드 입력만으로 로봇을 직관적으로 제어할 수 있도록 조작성 향상.

### 2.6. 순수 C++ 기반 2D 센서 퓨전(Sensor Fusion) 기능 구현
*   **문제**: 기존 ROS 2 시스템의 EKF(`robot_localization`)를 걷어내고 C++ 구조로 이관하면서 센서 퓨전 알고리즘이 누락됨. 그 결과 로봇이 공중에 떠 있어 바퀴만 회전하는 상황에서도 GUI 상의 맵에서는 로봇이 이동하는 것으로 표시되는 문제가 발생함.
*   **수정**:
    *   `pinky_core/include/pinky_core/core/sensor_fusion.h` 및 `pinky_core/src/core/sensor_fusion.cpp` 신규 작성.
    *   바퀴 오도메트리의 병진/회전 속도(v, w)와 BNO055 IMU의 자이로스코프(Yaw Rate) 데이터를 결합하는 **상보 필터(Complementary Filter)** 기반의 경량 2D 센서 퓨전 알고리즘 구현.
    *   가중치(`alpha_=0.9`)를 적용해 IMU의 회전 정보를 90% 신뢰하도록 설정함으로써, 바퀴 슬립이나 공회전 시에도 실제 로봇의 회전 각도를 정확히 유지하도록(바퀴 헛돎 방지) 시스템 안정성 극대화.

### 2.7. BNO055 IMU 통신 실패 시 시스템 프리징(Freezing) 방지
*   **문제**: 하드웨어 결함 또는 선 빠짐 등으로 BNO055 I2C 칩 ID(`0xA0`)를 정상적으로 읽지 못하는 상황에서도 `Bno055Imu::Init()` 내부의 통신 재시도 루프(while 문)로 진입하여 스레드가 블로킹됨. 이로 인해 터미널이 `Ctrl+C` 입력에 응답하지 않는 데드락 발생.
*   **수정**:
    *   `pinky_core/src/hal/bno055_imu.cpp` 파일 수정.
    *   `chipid`가 `0xA0`가 아닐 경우 경고 로그 출력 직후 즉시 `return false;` (Early Return) 하도록 로직 변경.
    *   하드웨어 오류 시 I2C 설정만 실패 처리하고 로봇 메인 프로세스는 멈춤 없이 다른 센서 및 구동 로직을 이어가도록 시스템 안정성 확보.

## 3. 결과 및 향후 계획
*   **검증**: 위 모든 수정 사항은 로컬 PC 상에서 단위 테스트(`pytest tests/`) 및 `make -j4` 빌드를 통과하였으며 동작 확인을 완료함.
*   **향후 계획**: 
    *   카메라 영상의 프레임 레이트 및 해상도 최적화.
    *   GUI 리팩토링 안정화 및 통합 시뮬레이션 환경(Gazebo)과의 오도메트리 동기화 테스트.