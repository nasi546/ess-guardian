#!/bin/bash
set -euo pipefail

LOG="/var/log/ess-camera-setup.log"
DEVICE="/dev/cam_rgb"
TIMEOUT=15

mkdir -p "$(dirname "$LOG")"
exec >> "$LOG" 2>&1

echo "[$(date)] [ess-camera-setup] start"

for i in $(seq 1 $TIMEOUT); do
  if [ -e "$DEVICE" ]; then
    echo "[$(date)] [ess-camera-setup] found $DEVICE"
    break
  fi
  echo "[$(date)] [ess-camera-setup] waiting for $DEVICE... ($i/$TIMEOUT)"
  sleep 1
done

if [ ! -e "$DEVICE" ]; then
  echo "[$(date)] [ess-camera-setup] SKIP: $DEVICE not found"
  exit 0
fi

echo "[$(date)] [ess-camera-setup] list formats:"
/usr/bin/v4l2-ctl -d "$DEVICE" --list-formats-ext || true

echo "[$(date)] [ess-camera-setup] try set 640x480 YUYV"
if ! /usr/bin/v4l2-ctl -d "$DEVICE" --set-fmt-video=width=640,height=480,pixelformat=YUYV; then
  echo "[$(date)] [ess-camera-setup] WARN: YUYV not supported, try MJPG"
  /usr/bin/v4l2-ctl -d "$DEVICE" --set-fmt-video=width=640,height=480,pixelformat=MJPG || true
fi

echo "[$(date)] [ess-camera-setup] set fps=30"
/usr/bin/v4l2-ctl -d "$DEVICE" --set-parm=30 || true

echo "[$(date)] [ess-camera-setup] dump all:"
/usr/bin/v4l2-ctl -d "$DEVICE" --all || true

echo "[$(date)] [ess-camera-setup] done"
