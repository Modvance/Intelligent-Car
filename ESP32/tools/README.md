# ESP32 辅助工具

本目录保存用于分析 ESP32 控制逻辑的辅助脚本。它们不需要烧录到 ESP32。

当前基础运动控制推荐使用 `../car_ctrl_esp32_pwm/`，本目录中的 PID 仿真工具主要用于后续继续研究 `../car_ctrl_esp32_gmr/`。

## 文件说明

- `esp32_pid_sim.py`
  - 根据 `motor_encoder_test` 的实测日志，粗略模拟 GMR 闭环控制趋势。
  - 用于排查 PID 参数是否可能导致 PWM 持续升高。
  - 不能替代真实小车测试，地面摩擦、电池电压、载重和轮胎打滑仍然需要实车确认。

## PID 仿真输入

仿真器默认读取：

```text
Car22/ESP32/motor_encoder_test/log.txt
```

这个文件通常来自烧录 `motor_encoder_test.ino` 后，在串口监视器中复制出来的测试日志。仓库里不一定提交该日志文件。

也可以手动指定日志路径：

```bash
python Car22/ESP32/tools/esp32_pid_sim.py --log Car22/ESP32/motor_encoder_test/log.txt --seconds 8 --motion forward --speed 14
```

## 常用仿真命令

默认前进仿真：

```bash
python Car22/ESP32/tools/esp32_pid_sim.py --seconds 8 --motion forward --speed 14
```

显式指定 PID 参数：

```bash
python Car22/ESP32/tools/esp32_pid_sim.py --seconds 8 --motion forward --speed 14 --kp 0.22 --ki 0.10
```

临时关闭积分项：

```bash
python Car22/ESP32/tools/esp32_pid_sim.py --seconds 8 --motion forward --speed 14 --ki 0.00
```

输出 CSV 方便画曲线：

```bash
python Car22/ESP32/tools/esp32_pid_sim.py --seconds 8 --motion forward --speed 14 --csv pid_debug.csv
```

## 输出字段

- `target_rpm`：ESP32 根据速度百分比换算出的目标轮速。
- `pwm@1s`：仿真 1 秒时的 PWM。
- `end_pwm`：仿真结束时的 PWM。
- `max_abs_pwm`：仿真过程中最大的 PWM 绝对值。
- `trend`：如果结束 PWM 明显大于 1 秒时 PWM，会显示 `up`。
