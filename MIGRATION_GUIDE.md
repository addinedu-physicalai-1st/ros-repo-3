# Pinky Project: Python to C++ Migration & Optimization Guide

이 문서는 Pinky 로봇의 제어 성능 향상과 시스템 안정성을 위해 진행된 Python 노드들의 C++ 마이그레이션 및 최적화 내역을 정리한 가이드입니다.

---

## 1. 개요
기존 Python 기반의 하위 드라이버 노드들을 C++(`rclcpp`)로 전환하여 다음과 같은 이점을 확보했습니다:
*   **지연 시간(Latency) 감소**: Python의 GIL(Global Interpreter Lock) 및 GC(Garbage Collection) 지연 제거.
*   **제어 주기 상향**: 기본 30Hz에서 **50Hz~100Hz**로 제어 주기 상향 가능.
*   **리소스 효율성**: 메모리 사용량 최적화 및 CPU 부하 감소.
*   **시스템 안정성**: 센서 데이터 처리에 비트 연산 및 하드웨어 가속 활용.

---

## 2. 마이그레이션 상세 내역

### 2.1. `pinky_bringup_cpp` (모터 제어 및 오도메트리)
*   **대상**: `pinky_bringup` (Python)
*   **주요 변경 사항**:
    *   `DynamixelSDK` C++ 라이브러리를 직접 연동하여 모터 통신 속도 개선.
    *   제어 루프 주기를 **50Hz**로 설정하여 즉각적인 모터 반응 확보.
    *   `publish_tf` 파라미터 추가: EKF 사용 시 `odom -> base_footprint` TF 발행 여부를 선택 가능하게 함.
    *   오도메트리 계산 로직 최적화 및 정밀도 향상.

### 2.2. `pinky_led_cpp` (LED 제어)
*   **대상**: `pinky_led` (Python)
*   **최적화 기법**:
    *   `rpi_ws281x` C 라이브러리를 직접 호출하여 하드웨어 제어.
    *   **비트마스킹**: RGB888 데이터를 32비트 ARGB 포맷으로 패킹할 때 비트 연산(`(R << 16) | (G << 8) | B`)을 사용하여 처리 속도 향상.
    *   서비스 콜백 내의 데이터 검증 로직 강화 (`std::clamp` 등 활용).

### 2.3. `pinky_emotion_cpp` (LCD 감정 표현)
*   **대상**: `pinky_emotion` (Python)
*   **최적화 기법**:
    *   **OpenCV 연동**: GIF 이미지 디코딩 및 프레임 관리에 OpenCV를 도입하여 이미지 처리 효율 극대화.
    *   **Pre-loading**: 모든 감정 GIF를 기동 시 메모리에 캐싱하여 런타임 지연 제거.
    *   **고속 포맷 변환**: RGB888에서 LCD 전용 RGB565로 변환 시 비트마스킹(`(R & 0xF8) << 8 | (G & 0xFC) << 3 | (B >> 3)`) 알고리즘 적용.

### 2.4. `pinky_sensor_adc` (기존 C++ 노드 최적화)
*   **최적화 내용**:
    *   OS 레벨의 `rclcpp::sleep_for` 대신 하드웨어 정밀 타이밍을 위한 `delayMicroseconds` 적용.
    *   12비트 ADC 결과 조립 시 비트 합집합(`|`) 연산으로 리팩토링하여 연산 효율성 및 가독성 개선.

---

## 3. 신규 도입: IMU 센서 퓨전 (EKF)
로봇의 위치 인식 정밀도를 높이기 위해 `robot_localization` 패키지를 도입했습니다.
*   **설정 파일**: `pinky_bringup_cpp/params/ekf.yaml`
*   **퓨전 데이터**: 바퀴 인코더 오도메트리(`odom`) + BNO055 IMU(`imu_raw`).
*   **효과**: 로봇의 제자리 회전 또는 급격한 가감속 시 발생하는 바퀴 미끄러짐 오차를 IMU의 자이로 데이터를 통해 실시간 보정.

---

## 4. 아키텍처 호환성 (PC vs 로봇)
PC(amd64) 환경에서도 빌드 에러 없이 시뮬레이션을 수행할 수 있도록 모든 C++ 패키지에 아키텍처 체크 로직을 적용했습니다.
*   **ARM64 (라즈베리 파이)**: 하드웨어 의존성이 있는 모든 노드(WiringPi, ws2811 등)를 빌드 및 설치.
*   **AMD64 (PC)**: 하드웨어 노드 빌드를 자동으로 스킵하여 Gazebo/RViz2 시뮬레이션 실습에 지장이 없도록 함.

---

## 5. 실행 방법

### 실기체 (라즈베리 파이)
```bash
ros2 launch pinky_bringup_cpp bringup.launch.py
```
*이 런치 파일은 C++ Bringup, IMU, EKF, Lidar, Battery 노드를 통합 실행합니다.*

### 시뮬레이션 (PC)
```bash
# Gazebo 실행
ros2 launch pinky_gz_sim launch_sim.launch.xml

# Navigation 실행
ros2 launch pinky_navigation gz_bringup_launch.xml
```

---
**작성일**: 2026-03-25
**작성자**: Gemini CLI
