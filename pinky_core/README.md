# Pinky Core - 순수 C++ 자율주행 로봇 실행 및 빌드 가이드

이 문서는 라즈베리 파이 5(또는 호환되는 ARM64 보드) 환경에서 **ROS 2 의존성 없이 순수 C++ 소스로만** 하드웨어를 제어하고 로봇을 구동하기 위한 초기 환경 구축 및 실행 방법을 안내합니다.

(주의: 이 가이드는 ROS 2 패키지가 포함된 기존 `pinky_pro` 폴더를 전혀 사용하지 않으며, 오직 `pinky_core`와 `pinky_devices` 폴더만을 대상으로 합니다.)

---

## 🚀 1. 로봇(라즈베리 파이) 필수 환경 구축하기

로봇의 터미널을 열고 아래 명령어를 순서대로 복사하여 실행하세요.

### 1-1. 시스템 기본 패키지 설치
영상 처리(OpenCV), 로그 쓰기(spdlog), 통신 툴(i2c-tools) 등 필수 시스템 라이브러리를 설치합니다.
```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config git wget unzip i2c-tools
sudo apt install -y libopencv-dev libspdlog-dev scons
```

### 1-2. ONNX Runtime 설치 (AI 자율주행 모델 추론용)
마이크로소프트의 공식 ARM64 컴파일 버전을 받아 시스템(/opt/onnxruntime)에 설치합니다.
```bash
cd ~
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-aarch64-1.17.1.tgz
tar -xvzf onnxruntime-linux-aarch64-1.17.1.tgz
sudo mv onnxruntime-linux-aarch64-1.17.1 /opt/onnxruntime
```

### 1-3. Dynamixel SDK 설치 (모터 구동용)
ROS 2 버전을 사용하지 않기 위해 인터넷에서 C++ 소스코드를 새로 받아 컴파일합니다. (라즈베리 파이는 ARM 보드이므로 반드시 `linux_sbc` 폴더에서 진행합니다)
```bash
cd ~
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux_sbc
make -j4
sudo make install
```

### 1-4. 순수 하드웨어 제어 라이브러리 빌드 (WiringPi & LED)
이 패키지들(WiringPi, rpi_ws281x)은 ROS와는 무관한 라즈베리 파이 단일 칩 전용 범용 C/C++ 하드웨어 드라이버입니다. 로봇으로 복사해 오신 `pinky_devices` 폴더 안에서 빌드를 진행합니다.
*(만약 폴더를 복사하지 않았다면 `git clone https://github.com/WiringPi/WiringPi` 로 받아도 됩니다)*

**WiringPi (GPIO, I2C, SPI 핀 통신용):**
```bash
cd ~/pinky_devices/WiringPi
./build
```

**rpi_ws281x (LED 및 조명 제어용):**
```bash
cd ~/pinky_devices/rpi_ws281x
scons
sudo mkdir -p /usr/local/include/ws2811
sudo cp *.h /usr/local/include/ws2811/
sudo cp libws2811.a /usr/local/lib/
```

**Slamtec RPLIDAR SDK (라이다 센서 통신용):**
로봇의 메인 눈 역할을 하는 라이다 SDK도 로컬로 빌드해 줍니다.
```bash
cd ~/pinky_devices
git clone https://github.com/Slamtec/rplidar_sdk.git
cd rplidar_sdk
make
```

### 1-5. 장치 권한 부여 (매우 중요 ⭐)
프로그램이 `sudo` 관리자 권한 없이도 모터(Serial), IMU(I2C) 등을 자유롭게 제어할 수 있도록 계정에 권한을 줍니다. 
(OS 버전에 따라 특정 그룹이 없다는 에러가 나면 해당 단어는 지우고 입력하세요)
```bash
sudo usermod -a -G dialout,i2c,gpio $USER
```
> **❗ 주의:** 권한 부여 후 반드시 시스템을 **재부팅(Reboot)**해야 권한이 정상 적용됩니다! 
> (또한, 워터펌프/LED 등 일부 하드웨어는 결국 `sudo ./pinky_robot` 실행이 강제될 수 있습니다.)

---

## 🚀 2. 순수 C++ 코어(Pinky Core) 빌드 및 실행

모든 환경 설정과 재부팅이 끝났다면, 로봇의 메인 두뇌인 `pinky_core`를 빌드합니다.

```bash
cd ~/pinky_core
mkdir build && cd build

# CMake를 통해 Make파일 생성
cmake ..

# 빌드 (라즈베리 파이 5 기준 -j4 추천)
make -j4
```

성공적으로 빌드가 완료되면 `pinky_robot` 이라는 실행 파일이 생성됩니다. 
이를 실행하면 9100(TCP), 9200(UDP) 포트를 열고 연결 대기 상태에 돌입합니다.

```bash
# 프로그램 실행
./pinky_robot

# 만약 "/dev/ttyAMA0 권한 없음" 등의 에러가 발생한다면 권한이 덜 풀린 것이므로 아래와 같이 실행합니다.
sudo ./pinky_robot
```

---

## 🚀 3. PC 관제탑(Pinky Station)에서 핑키 조종하기

로봇이 성공적으로 실행되어 센서를 돌리고 있다면, 이제 PC 노트북 등에서 원격으로 핑키에 접속해 상태를 확인합니다.

1. PC에서 필요한 파이썬 라이브러리를 설치합니다.
   ```bash
   pip install PyQt6 pyqtgraph numpy
   ```
2. PC에 복사된 `pinky_station` 폴더로 이동합니다.
   ```bash
   cd ~/pinky_station
   ```
3. 관제탑 GUI 프로그램을 실행합니다.
   ```bash
   python main.py
   ```
4. 화면 상단에 핑키의 IP 주소(예: `192.168.0.x`)를 입력하고 **Connect** 버튼을 클릭합니다.
5. 터미널 창에 "Connected safely."가 뜨고 배터리 전압 수치 등이 움직인다면 연결이 완벽히 성공한 것입니다!
