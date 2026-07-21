# ESP32 Arduino 程序

本目录保存烧录到 ESP32 IOT & Robot Board 的 Arduino 程序，用于控制四个 WHEELTEC GMR 编码器 TT 马达。

ESP32 通过 USB 串口与 Atlas200I DKA2 通信，接收华为官方 Python 端兼容的 14 字节二进制控制包：

```text
[M1, M2, M3, M4, Servo1, Servo2, -12345]
```

其中 `M1..M4` 是四个电机命令，范围建议控制在 `-100..100`。

## 当前推荐烧录

当前基础运动控制建议烧录：

```text
car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino
```

原因：

- 与 `../base/main.py` 和 `../ros2/ros2_ws` 的串口协议兼容。
- 不依赖编码器 PID，行为更直接，适合作为当前基础功能保底。
- 带一次性启动脉冲，低速启动时更容易突破静摩擦。
- 已用于当前手动控制和运动测试流程。

两套 Atlas 实现都发送相同的 14 字节帧，因此 ESP32 不需要随 base/ROS2 切换重新烧录。

## 程序说明

### `motor_encoder_test/motor_encoder_test.ino`

用于首次接线和编码器检查。

在 Arduino IDE 串口监视器中选择 `115200` 波特率，可输入：

```text
help
auto
m1 25
m1 -25
all 25
stop
rpm
```

用途：

- 验证每个电机是否能正反转。
- 验证编码器 A/B 相是否有读数。
- 判断 `MOTOR_SIGN` 或编码器接线方向是否需要调整。

第一次测试必须架空小车。

### `car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino`

当前推荐的基础运动版本。

特点：

- 纯 PWM 输出，不读取编码器，不执行 PID。
- Atlas 发来的电机命令会作为 PWM 百分比输出。
- 串口返回 `SUCC` 或 `FAIL`，与 Atlas Python 端控制逻辑兼容。
- 从停止到运动或方向反转时，会短暂输出启动脉冲，然后自动回到目标 PWM。
- 已集成双 MG90S 摄像头云台：GPIO25 控制 Pan，GPIO26 控制 Tilt。
- 云台上电只回到标定位置 `(93, 162)`，不会自动巡检。
- Pan 范围为 `0..180`，Tilt 范围为 `90..162`；超出范围的 Atlas 命令会在 ESP32 端自动裁剪。

常用参数：

```cpp
const int8_t MOTOR_SIGN[WHEEL_COUNT] = {1, 1, 1, 1};
const int16_t START_KICK_PWM = 50;
const unsigned long START_KICK_MS = 120;
const int16_t PAN_CENTER = 93;
const int16_t TILT_CENTER = 162;
```

调参建议：

- 某个轮子方向反了：只改对应的 `MOTOR_SIGN` 为 `-1`。
- 低速启动不了：适当增大 `START_KICK_PWM` 或 `START_KICK_MS`。
- 启动冲一下太明显：适当减小 `START_KICK_PWM` 或 `START_KICK_MS`。

### `car_ctrl_esp32_gmr/car_ctrl_esp32_gmr.ino`

GMR 编码器闭环实验版本。

特点：

- 使用编码器测速并尝试 PID 闭环速度控制。
- 保留用于后续继续研究闭环控制。
- 当前基础提交不建议作为默认烧录版本。

当前参数按 WHEELTEC GMR 编码器 TT 马达配置：

```cpp
ENCODER_PULSES_PER_MOTOR_REV = 500
GEAR_RATIO = 48.0f
MAX_WHEEL_RPM = 330.0f
```

如果使用 1:90 减速比马达，需要把 `GEAR_RATIO` 改成 `90.0f`，并把 `MAX_WHEEL_RPM` 调到约 `165..175`。

## Arduino IDE 设置

1. 安装 ESP32 开发板支持包。
2. 打开需要烧录的 `.ino` 文件。
3. 选择对应 ESP32 开发板型号和串口。
4. 点击编译并上传。

这些程序除了各自目录内的本地 `.cpp/.h` 文件外，不依赖额外 Arduino 第三方库。

## 电机方向说明

四轮小车在 Atlas 端前进命令中，电机值通常不是四个同号，而是类似：

```text
[-speed, -speed, speed, speed]
```

这是由底盘安装方向决定的。当前建议：

- Atlas 端 `motor_rating` 保持正比例，例如 `[1, 1, 1, 1]`。
- 具体某个电机方向反了，在 ESP32 的 `MOTOR_SIGN` 中修正。
- 不建议在 Atlas 端把某个轮子的 `motor_rating` 改成负数来修方向。

## 编码器方向说明

GMR 编码器的 A/B 相用于判断转速和方向：

- A/B 相只影响 `motor_encoder_test` 和 `car_ctrl_esp32_gmr`。
- `car_ctrl_esp32_pwm` 不读取编码器，因此 A/B 相不会影响当前 PWM 基础版本。
- 如果闭环版里某个轮子的测速正负号反了，修改对应 `ENCODER_SIGN`。

## 辅助工具

`tools/` 中保留 PID 仿真脚本，主要用于后续继续研究闭环控制趋势。当前基础运动版本不依赖这个工具。
