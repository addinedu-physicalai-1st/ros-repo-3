# Phase 4.10: GUI 실기체 구동 버그 수정 보고서

> 작성일: 2026-04-02
> 대상: 로봇 실기체(`pinky_core`) 구동 및 PC GUI(`pinky_station`) 연결 후 발견된 4가지 기능 불가 버그 수정

---

## 1. 수정 내역 요약 (총 5건)

### #1. WASD 키보드 모터 제어 불가 — AttributeError

**파일:** `pinky_station/pinky_station/gui/widgets/teleop_widget.py`

**문제:**
`TeleopWidget.keyPressEvent()`와 `keyReleaseEvent()`에서 `self._pressed_keys` 집합을 참조하지만,
`__init__()` 어디에서도 초기화하지 않았음. 첫 번째 키 입력 시 즉시 `AttributeError`가 발생하여
이후 키보드 이벤트가 전혀 처리되지 않음.

추가로 `TeleopWidget`의 부모 클래스인 `QGroupBox`는 기본 포커스 정책이 `NoFocus`이므로,
키보드 이벤트 자체가 위젯에 전달되지 않는 구조적 문제도 함께 존재.

```python
# 수정 전 — __init__에 _pressed_keys 없음
def keyPressEvent(self, event):
    ...
    self._pressed_keys.add(key)  # AttributeError 발생
```

**수정:**
- `__init__()` 초기에 `self._pressed_keys: set = set()` 추가
- `self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)` 추가하여 위젯이 키보드 이벤트를 수신하도록 설정

```python
# 수정 후
self._pressed_keys: set = set()
self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
```

---

### #2. 센서 퓨전 IMU 미연동 — 순수 오도메트리와 동일하게 동작

**파일:** `pinky_core/src/app/robot_app.cpp`

**문제:**
Phase 4.9에서 `SensorFusion` 클래스(상보 필터)가 구현되었으나, `ImuLoop`에서 BNO055로부터
Yaw Rate(`angular_velocity.z`)를 읽은 뒤 UDP 전송만 하고 `sensor_fusion_.UpdateImu()`를
**호출하지 않음**. 결과적으로 `SensorFusion::has_imu_`가 항상 `false`인 채로 유지되어
`Predict()` 내부에서 보정 없이 휠 오도메트리만 그대로 적분됨.

로봇을 공중에 들고 바퀴만 회전시켜도 GUI 맵에 이동 경로가 그려지는 현상이 바로 이 원인임.

```cpp
// 수정 전 — IMU 데이터를 센서 퓨전에 반영하지 않음
if (imu_->ReadData(data)) {
    // sensor_fusion_.UpdateImu(...)  ← 없음
    udp_->Send(udp_pkt);
}
```

**수정:**
`imu_->ReadData()` 성공 직후 `state_mutex_` 잠금 하에 `sensor_fusion_.UpdateImu()`를 호출.
`MotorOdomLoop`의 `Predict()` 역시 동일 뮤텍스를 사용하므로 스레드 안전성 확보.

```cpp
// 수정 후
if (imu_->ReadData(data)) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sensor_fusion_.UpdateImu(data.angular_velocity.z, data.stamp);
    }
    udp_->Send(udp_pkt);
}
```

보정 가중치(`alpha_ = 0.9`)에 따라 IMU Yaw Rate를 90%, 휠 오도메트리를 10% 신뢰하여
바퀴 슬립·공회전 시 각도 오차를 억제함.

---

### #3. 맵 뷰포트가 로봇을 추종 — 절대 좌표 경로 확인 불가

**파일:** `pinky_station/pinky_station/gui/widgets/map_widget.py`

**문제:**
`paintEvent()`에서 월드→화면 변환 기준점 `(tx, ty)`를 아래와 같이 계산:

```python
# 수정 전
tx = cx + (self.offset_x - self.robot_x) * self.scale_factor
ty = cy - (self.offset_y - self.robot_y) * self.scale_factor
```

이 수식은 로봇 좌표 `(robot_x, robot_y)`를 뺌으로써 로봇이 항상 화면 중앙`(cx, cy)`에
고정되도록 강제함. 로봇이 이동해도 화면에서는 로봇 위치가 고정되고 월드(축·궤적)가 이동하는 구조로,
사용자가 절대 이동 경로를 시각적으로 파악할 수 없음.

Phase 4.9 문서에서 "월드 원점 기준으로 변경"이라고 기록되었으나 실제 코드에 반영되지 않은 상태였음.

**수정:**
뷰포트 기준점을 사용자 패닝 오프셋(`offset_x`, `offset_y`)만으로 결정하도록 수정.
로봇 좌표는 변환 결과에 더해지므로, 로봇이 월드를 이동하면 화면상에서도 실제로 움직임.

```python
# 수정 후
tx = cx + self.offset_x * self.scale_factor
ty = cy - self.offset_y * self.scale_factor
```

- 화면 중앙(offset = 0일 때): 월드 원점 (0,0) 고정
- 로봇은 `rx_screen = tx + robot_x * scale` 위치에 이동하며 표시
- 마우스 좌클릭 월드 좌표 역산식(`(screen - cx) / scale - offset`)은 새 변환과 일치하므로 수정 불필요

---

### #4. 카메라 최초 프레임 미표시 — null pixmap 설정

**파일:** `pinky_station/pinky_station/gui/widgets/video_view.py`

**문제:**
`update_frame()`에서 수신한 JPEG를 `QLabel`에 표시할 때 레이블의 현재 픽셀 크기로 스케일링:

```python
scaled_pixmap = pixmap.scaled(
    self.lbl_image.width(),   # 레이아웃 미확정 시 0 반환 가능
    self.lbl_image.height(),
    ...
)
self.lbl_image.setPixmap(scaled_pixmap)
```

GUI가 처음 표시되기 전(레이아웃 계산 전)에 첫 번째 프레임이 도착하면
`lbl_image.width() == 0, height() == 0`이어서 `pixmap.scaled(0, 0, ...)` → null `QPixmap` 반환.
null pixmap을 `setPixmap()`에 전달하면 레이블이 비워지고 아무것도 나타나지 않음.
이후 프레임이 연속 수신되더라도 동일 경로를 타면 반복 실패할 수 있음.

**수정:**
크기 검사를 추가하여 유효한 경우에만 스케일링하고, 크기가 0이면 원본 pixmap을 직접 설정.

```python
w = self.lbl_image.width()
h = self.lbl_image.height()
if w > 0 and h > 0:
    scaled_pixmap = pixmap.scaled(w, h, ...)
    self.lbl_image.setPixmap(scaled_pixmap)
else:
    self.lbl_image.setPixmap(pixmap)  # 레이아웃 확정 후 Qt가 자동 조정
```

---

### #5. 라이다 빈 배열 처리 오류 + requirements 누락

**파일:** `pinky_station/pinky_station/gui/widgets/lidar_view.py`,
`pinky_station/requirements.txt`

**문제 (1) — 빈 배열 처리:**
유효 포인트가 없을 때(`x_pts = []`) 아래와 같이 호출:

```python
self.scatter.setData(x=np.array([]), y=np.array([]))
```

`np.array([])` 는 shape `(0,)` 배열임. `ScatterPlotItem.setData`는 내부적으로
`np.empty((len(x), 2))` 형태의 좌표 행렬을 구성하는데, len == 0 이어도 행렬 생성 자체는 되나
이후 컬럼 슬라이싱 코드가 빈 배열에 대해 일관성 없이 동작하여 scatter 상태가 초기화되지 않고
잔여 데이터가 남을 수 있음. pyqtgraph 버전에 따라 예외 없이 무시되거나 화면 업데이트가 누락됨.

**문제 (2) — requirements 누락:**
`lidar_view.py`는 `import pyqtgraph as pg`를 사용하지만 `requirements.txt`에
`pyqtgraph`가 없어 신규 환경에서 `ImportError`로 GUI 전체가 시작되지 않음.

**수정:**
- 유효 포인트가 있을 때만 `setData` 호출, 없을 때는 `scatter.clear()` 사용
- `np.asarray(..., dtype=np.float64)` 로 dtype 명시

```python
if x_pts:
    self.scatter.setData(
        x=np.asarray(x_pts, dtype=np.float64),
        y=np.asarray(y_pts, dtype=np.float64),
    )
else:
    self.scatter.clear()
```

- `requirements.txt`에 `pyqtgraph>=0.13.0` 추가

---

## 2. 변경 파일 목록

| 파일 | 변경 유형 | 관련 수정 |
|------|-----------|-----------|
| `pinky_core/src/app/robot_app.cpp` | 기능 추가 | #2 |
| `pinky_station/pinky_station/gui/widgets/teleop_widget.py` | 버그 수정 | #1 |
| `pinky_station/pinky_station/gui/widgets/map_widget.py` | 버그 수정 | #3 |
| `pinky_station/pinky_station/gui/widgets/video_view.py` | 버그 수정 | #4 |
| `pinky_station/pinky_station/gui/widgets/lidar_view.py` | 버그 수정 | #5 |
| `pinky_station/requirements.txt` | 의존성 추가 | #5 |

---

## 3. 영향 및 검증

### 수정 후 예상 동작

| 기능 | 수정 전 | 수정 후 |
|------|---------|---------|
| WASD 키보드 제어 | `AttributeError` 즉시 크래시 | 정상 동작 (누르는 동안 이동, 떼면 정지) |
| 센서 퓨전 | 순수 휠 오도메트리 (슬립 미보정) | IMU Yaw Rate 90% 반영 상보 필터 |
| 맵 경로 표시 | 로봇 고정·월드 이동 (상대 좌표) | 로봇 이동·원점 고정 (절대 좌표) |
| 카메라 피드 | 최초 프레임 표시 실패 가능 | 레이아웃 미확정 시에도 표시 |
| 라이다 그래프 | 빈 데이터 시 상태 불일치 | `clear()`로 명시적 초기화 |

### 로봇 재빌드 필요

`robot_app.cpp` 수정 사항은 로봇에서 재빌드 필요:

```bash
cd ~/pinky_core/build
make -j4
```

### Python 의존성 재설치 (신규 환경)

```bash
cd ~/ros2_ws/pinky_cpp
source .venv/bin/activate
pip install -r pinky_station/requirements.txt
```

---

## 4. 남은 사항

- [ ] IMU 없는 환경(`imu_ == nullptr`)에서 센서 퓨전 fallback 동작 검증
- [ ] 실기체에서 센서 퓨전 적용 후 직진 오차 / 회전 오차 계측 및 `alpha_` 튜닝
- [ ] 카메라 프레임레이트 최적화 (현재 10fps 고정)
- [ ] 라이다 실측 데이터 기반 맵 렌더링 (현재 24-sector 포인트 클라우드)
