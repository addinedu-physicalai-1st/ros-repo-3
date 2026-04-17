# Phase 4-8 런타임 버그 수정 보고서

> 작성일: 2026-04-01
> 대상: PC 시뮬레이션 및 로봇 실기체(RPi5) 런타임 환경에서 발생한 연결/표시 버그 수정

---

## 1. 수정 내역 요약 (총 5건)

### #1. PC 스테이션 GUI 강제 종료 (AttributeError)

**파일:** `pinky_station/pinky_station/gui/main_window.py`

**문제:**
TCP/UDP 통신 연결 실패나 타임아웃 발생 시, 내부적으로 워커 스레드를 정리하는 `_cleanup_workers()` 함수가 호출됨. 이때 클래스 멤버로 선언되기 전인 `self.nav_worker` 등의 속성을 참조하려다 `AttributeError` 예외가 발생하며 데스크톱 앱이 크래시됨.

**수정:**
`getattr(self, 'nav_worker', None)`와 같이 `getattr` 내장 함수를 사용하여 객체의 초기화 상태를 안전하게 검증한 뒤에만 스레드의 `stop()` 및 `wait()`를 호출하도록 방어 코드를 추가했습니다.

---

### #2. Sllidar 라이다 하드웨어 인식 및 통신 실패

**파일:**
- `include/pinky_core/app/robot_app.h`
- `src/app/config_loader.cpp`
- `src/app/robot_app.cpp`

**문제:**
실기체 터미널에서 `SllidarDriver: Failed to get device info, code: 0x80008002` 에러 발생. 
`robot_config.yaml` 파일에 `hal.lidar` 설정으로 기체에 맞는 포트(`/dev/ttyAMA0`)와 보드레이트(`256000`)가 정의되어 있으나, C++ `RobotApp` 초기화 과정에서 이를 무시하고 빈 `SllidarDriver::Config{}` 기본값(`/dev/ttyUSB0`, 115200)을 넘겨서 타임아웃이 발생함.

**수정:**
`config_loader.cpp`에 `hal.lidar.device` 및 `baudrate` 파싱 로직을 추가하여 `RobotConfig` 구조체에 주입하고, `robot_app.cpp`에서 Lidar 드라이버를 생성할 때 파싱된 실제 파라미터를 넘겨주도록 수정했습니다.

---

### #3. LCD 색상 반전 및 감정 GIF 애니메이션 재생 멈춤

**파일:**
- `src/hal/ili9341_lcd.cpp`
- `CMakeLists.txt`
- `src/core/emotion_renderer.cpp`

**문제:**
1. **색상 오류:** 검은색 배경이 흰색으로, 분홍색 감정 아이콘이 녹색으로 출력됨. ILI9341 패널 초기화 시 색상 반전 제어 설정이 누락된 것이 원인.
2. **애니메이션 멈춤:** 화면이 움직이지 않고 1프레임 정지 이미지로만 나옴. 기존의 `stb_image` 라이브러리(`stbi_load_gif_from_memory`)가 용량이 크거나 특수한 로컬 컬러 테이블을 사용하는 GIF 디코딩에 실패하여, 도형 렌더링 폴백(Fallback) 모드나 단일 이미지 모드로 전환되어 발생.

**수정:**
1. `ili9341_lcd.cpp` 초기화 시퀀스(MADCTL 직후)에 `0x21` (Display Invert ON) 명령어를 추가하여 색상 반전을 해결함.
2. `CMakeLists.txt`에 OpenCV(`videoio`, `imgproc`) 의존성을 연결하고, `emotion_renderer.cpp`에 `cv::VideoCapture` 기반의 완벽한 다중 프레임 디코더를 구현. 모든 프레임을 읽어와서 ILI9341 네이티브 포맷인 RGB565 리틀엔디안으로 변환하도록 최적화함. (OpenCV 비활성화 시 기존 `stb` 모드로 동작하도록 매크로 처리)

---

### #4. 전진/후진 (W/S) 입력 무시 및 다이나믹셀 모터 정지 현상

**파일:**
- `include/pinky_core/hal/dynamixel_motor.h`
- `src/hal/dynamixel_motor.cpp`

**문제:**
조향(A/D) 명령은 수행되나, 병진(W/S) 명령을 보내면 바퀴가 전혀 굴러가지 않는 현상. 
확인 결과, C++ 하드웨어 드라이버 내부에서 모터 회전 속도(RPM) 값을 입력받아 처리하는 `SetVelocityWait` 함수가 인자의 단위를 `rad/s`로 착각하고 변환 계수(`rpm_to_rads_`)로 한 번 더 나누는 이중 계산(Double Conversion) 버그가 있었습니다. 이로 인해 정상 RPM이 수천 RPM으로 증폭되어 다이나믹셀 모터의 내부 안전장치(Max Speed Limit)가 작동, 회전을 강제 차단했습니다.

**수정:**
`dynamixel_motor.cpp` 및 헤더 파일에서 `SetVelocityWait` 함수의 매개변수를 직관적인 `rpm`으로 수정하고, 함수 내부에 존재하던 불필요한 이중 나눗셈 로직을 완전히 제거했습니다.

---

### #5. 라이다 C1 통신 불가 (0x80008004 에러) 및 순수 C++ 환경 구축

**파일:**
- `CMakeLists.txt`
- `src/app/main.cpp`
- `include/pinky_core/app/robot_app.h`
- `pinky_devices/rplidar_sdk/` (신규 분리)

**문제:**
1. 라이다 장치와 통신 연결 후 `0x80008004 (OPERATION_NOT_SUPPORT)` 에러가 발생. 이는 구버전 라이다 라이브러리가 C1 기종의 460800 보드레이트와 새로운 통신 규격을 파싱하지 못해서 발생. 
2. 기존 시스템에 남아있던 구버전 정적 라이브러리(`.a`)를 잘못 링크하고 있었으며, 앱 실행 시 `--config` 파라미터를 넘겨주지 않으면 C++ 코어에 하드코딩된 기본값 `115200`으로 통신을 시도함.
3. ROS(로봇 운영체제) 패키지 종속성 없이 순수 C++ 단일 환경으로 동작해야 하는 요구사항을 충족하지 못함.

**수정:**
1. **ROS 의존성 제거 및 SDK 독립:** `sllidar_ros2` 내부에 있던 최신 RPLiDAR SDK 소스코드를 ROS 패키지 밖인 `pinky_devices/rplidar_sdk`로 전면 분리하고, `CMakeLists.txt`가 구버전 라이브러리 대신 이 최신 C/C++ 소스코드를 직접 가져와 함께 컴파일하도록 수정했습니다.
2. **자동 설정 파일 로드:** `main.cpp`를 수정하여, 사용자가 앱 실행 시 `--config` 옵션을 명시하지 않더라도 프로그램이 자동으로 `../config/robot_config.yaml` 파일을 찾아서 파싱하도록 기본 동작을 개선했습니다.
3. **C1 보드레이트 기본값 반영:** `robot_app.h`의 Lidar 기본 보드레이트 값을 `115200`에서 C1 기준인 `460800`으로 하드코딩 교체하여, 어떠한 경우에도 최우선적으로 C1 장비를 스캔하도록 조치했습니다.

---

## 2. 하드웨어 런타임 검증 현황 업데이트

기존 `phase4_8_remaining_impl_report.md`에 명시되었던 미검증 항목 중 금번 런타임 테스트를 통해 수정 및 검증 완료된 사항입니다:

* [x] SllidarDriver RPLiDAR SDK 런타임 검증 (성공)
* [x] ILI9341 LCD + EmotionRenderer 실물 표시 검증 (애니메이션 정상 구동)
* [x] DynamixelMotor 실 하드웨어 검증 (W/S 전후진 구동 성공)
