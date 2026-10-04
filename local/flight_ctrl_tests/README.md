# flight_ctrl_tests

Pi 上可整夹删除的台架脚本（仓库副本在本目录）。走扇出 `udpin:127.0.0.1:14550`，不碰 `/dev/ttyAMA0`。
**解锁/上锁只用遥控器**，脚本不发 arm。跑之前停掉 `interface_bringup`（会占 14550）。

```bash
/home/wly/dronelink-venv/bin/python3 ~/flight_ctrl_tests/bench_motors.py
/home/wly/dronelink-venv/bin/python3 ~/flight_ctrl_tests/bench_drop.py
/home/wly/dronelink-venv/bin/python3 ~/flight_ctrl_tests/check_ekf_gps.py
```

- `bench_motors.py`：只打印 HEARTBEAT 的 armed；你遥控解锁看电机，再上锁。
- `bench_drop.py`：`MAV_CMD_DO_SET_SERVO` → **SERVO9(=AUX1)**，1100µs 约 2 s 再 1500 停。
  不是 RC 通道 9 override（RC9 ≠ SERVO9；MAVLink1 override 也到不了 CH9）。
  台架飞控：`SERVO9_FUNCTION=0`；AUX 舵机单独供电、与飞控共地。
  打印 `ack=0` 为 ACCEPTED；`DENIED` 检查 FUNCTION / 供电。
- `check_ekf_gps.py`：只读 EKF/GPS（`EKF_POS_HORIZ_ABS|EKF_POS_VERT_ABS`、fix、sats）。不解锁。地面 HUD 同源字段由 `telem_ground.py` 推。
