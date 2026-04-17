#!/bin/bash
# =============================================================
# Pinky Robot - Gazebo Standalone Launcher
# Gazebo를 ROS2 launch 없이 독립 실행하고 로봇을 스폰합니다.
# ros_bridge.launch.py 와 함께 사용하세요.
# =============================================================

set -e

echo "=== [1/4] 기존 Gazebo 프로세스 정리 중... ==="
pkill -9 gz 2>/dev/null || true
pkill -9 ruby 2>/dev/null || true
sleep 1

echo "=== [2/4] ROS2 환경 설정 ==="
source ~/ros2_ws/pinky/pinky_pro/install/setup.bash

# GZ_SIM_RESOURCE_PATH: Gazebo가 모델/메시 파일을 찾는 경로
DESC_SHARE=$(ros2 pkg prefix pinky_description)/share
GZ_SHARE=$(ros2 pkg prefix pinky_gz_sim)/share

export GZ_SIM_RESOURCE_PATH="${DESC_SHARE}/pinky_description/../:${GZ_SHARE}/pinky_gz_sim/models:${HOME}/.gazebo/models"

# 시스템 플러그인(램프 제어 등)을 찾기 위한 경로 추가
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(ros2 pkg prefix pinky_gz_sim)/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"

WORLD_FILE="${GZ_SHARE}/pinky_gz_sim/worlds/pinky_factory.world"
echo "  월드 파일: ${WORLD_FILE}"

echo "=== [3/4] Gazebo 서버 + GUI 실행 ==="
# -s: 서버(물리엔진)만, -r: 즉시 시뮬 시작, -v4: 상세 로그
gz sim -s -r -v4 "${WORLD_FILE}" &
GZ_SERVER_PID=$!
echo "  Gazebo 서버 PID: ${GZ_SERVER_PID}"

sleep 4  # 서버가 완전히 뜰 때까지 대기

# -g: GUI만 실행 (서버와 분리하면 GUI 크래시 시 서버는 유지됨)
gz sim -g -v4 &
GZ_GUI_PID=$!
echo "  Gazebo GUI PID: ${GZ_GUI_PID}"

sleep 2

echo "=== [4/4] 로봇 스폰 ==="
# /robot_description 토픽에서 URDF를 읽어 Gazebo에 spawning
ros2 run ros_gz_sim create \
  -name pinky \
  -topic /robot_description \
  -x 0.0 -y 0.0 -z 0.1

echo ""
echo "============================================="
echo " Gazebo + Pinky 로봇 준비 완료!"
echo " 이제 별도 터미널에서 실행하세요:"
echo "   ros2 launch pinky_gz_sim ros_bridge.launch.py"
echo "============================================="
echo ""
echo " 종료하려면 Ctrl+C (또는 Gazebo GUI 닫기)"

# Gazebo 서버가 종료될 때까지 대기
wait ${GZ_SERVER_PID}
