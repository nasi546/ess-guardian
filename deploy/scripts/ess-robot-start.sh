#!/usr/bin/env bash
set -e

LOGFILE="/var/log/ess-robot.log"

# 로그 파일로 stdout/stderr 다 보내기
mkdir -p "$(dirname "$LOGFILE")"
exec >> "$LOGFILE" 2>&1

echo "[$(date)] ess-robot-start.sh: starting TurtleBot3 bringup"

# ===== ROS 환경 설정 =====
# ROS2 Humble
source /opt/ros/humble/setup.bash

# turtlebot3_ws
if [ -f /home/ros/turtlebot3_ws/install/setup.bash ]; then
    source /home/ros/turtlebot3_ws/install/setup.bash
else
    echo "[$(date)] ERROR: /home/ros/turtlebot3_ws/install/setup.bash not found"
fi

# ===== 공통 ROS 환경 변수 =====
export ROS_DOMAIN_ID=37
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# LDS 모델 (필요한 걸로 수정)
export LDS_MODEL=LDS-02    # LDS-02, LDS-03 쓰면 바꾸기

# 터틀봇 모델 (네가 가진 모델로 수정)
export TURTLEBOT3_MODEL=burger   # waffle_pi면 waffle_pi

# ===== OpenCR (모터 보드) 디바이스 체크 =====
if [ ! -e /dev/ttyACM0 ]; then
    echo "[$(date)] WARNING: /dev/ttyACM0 not found. OpenCR not connected?"
fi

echo "[$(date)] ess-robot-start.sh: env ready, launching robot.launch.py"

# ===== TurtleBot3 bringup 실행 =====
exec ros2 launch turtlebot3_bringup robot.launch.py

