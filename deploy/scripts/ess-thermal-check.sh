
#!/bin/bash
set -euo pipefail

LOG="/var/log/ess-thermal-check.log"
exec >>"$LOG" 2>&1

BUS=9
ADDR_CELL_EXPECT="33"

echo "[$(date)] ess-thermal-check: start (bus=$BUS, addr=0x33)"

# i2cdetect 테이블에서 0x33은 row 0x30의 4번째 칸(= $5)
cell="$(sudo i2cdetect -y "$BUS" | awk '$1=="30:"{print $5}')"

if [[ "$cell" == "$ADDR_CELL_EXPECT" ]]; then
  echo "[$(date)] OK: MLX90640 detected at 0x33 on i2c-$BUS"
  exit 0
fi

echo "[$(date)] ERROR: MLX90640 not detected (cell=$cell). Check wiring/I2C/baudrate."
exit 1
