# Atlas200I DKA2 智能小车基础代码

本目录是当前小车项目的最小可运行代码，用于 Atlas200I DKA2 开发板配合 ESP32 IOT & Robot Board 控制 WHEELTEC GMR 编码器 TT 马达。

当前阶段的重点是先稳定完成基础运动链路：Atlas Python 程序通过 USB 串口向 ESP32 发送电机命令，ESP32 驱动四个 TT 马达完成前进、后退、转向、旋转和停止。

## 当前状态

- 当前推荐烧录 `ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino` 作为基础运动版本。
- `car_ctrl_esp32_pwm` 使用华为官方 Python 端兼容的 14 字节二进制串口协议，不依赖编码器闭环 PID。
- 低速启动时使用一次性启动脉冲，解决小 PWM 下轮子克服不了静摩擦的问题。
- `ESP32/car_ctrl_esp32_gmr` 保留为 GMR 编码器闭环实验版本，目前不作为基础运动保底方案。
- Atlas 端手动控制和最小运动测试位于 `python/`。
- 摄像头远程调试工具位于 `camera/`。

## 目录结构

```text
Car22/
├─ ESP32/
│  ├─ car_ctrl_esp32_pwm/     # 当前推荐烧录的纯 PWM 基础运动版本
│  ├─ car_ctrl_esp32_gmr/     # GMR 编码器闭环实验版本
│  ├─ motor_encoder_test/     # 单独测试电机和编码器接线
│  └─ tools/                  # ESP32 相关的 PID 仿真等辅助工具
├─ python/
│  ├─ main.py                 # Atlas 端小车程序入口
│  ├─ motion_test.py          # Atlas 到 ESP32 的最小运动测试脚本
│  ├─ requirements.txt        # Atlas Python 依赖版本
│  └─ src/                    # 动作、场景、串口、相机和模型相关代码
└─ camera/
   └─ camera_debug_stream.py  # SSH 场景下的摄像头画面调试工具
```

## 快速开始

1. 先把小车架空，避免方向或速度配置错误导致小车冲出去。
2. 用 Arduino IDE 烧录当前基础运动版本：

```text
ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino
```

3. 在 Atlas 开发板上进入 Python 目录并安装依赖：

```bash
cd /home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22/python
python3 -m pip install -r requirements.txt
```

4. 检查 Atlas 是否识别到 ESP32 串口：

```bash
python3 motion_test.py --list-ports
```

5. 架空测试基础动作：

```bash
python3 motion_test.py --mode sequence --speed 25 --duration 0.8
```

6. 运行手动控制：

```bash
python3 main.py
```

## 手动控制键位

```text
w       前进
s       后退
a       左转
d       右转
q       逆时针旋转
e       顺时针旋转
space   停止
esc     退出
```

## 摄像头调试

小车没有显示器时，可以在小车端启动 MJPEG 服务，本机通过 SSH 端口转发查看画面：

```bash
cd /home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22
python3 camera/camera_debug_stream.py --camera 0 --max-camera-index 5 --width 1280 --height 720 --fps 30 --port 8080 --crosshair
```

本机另开终端：

```bash
ssh -L 8080:127.0.0.1:8080 root@小车IP
```

浏览器打开：

```text
http://127.0.0.1:8080
```

## 关键配置

- `ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino`
  - `MOTOR_SIGN`：修正单个轮子的驱动方向。
  - `START_KICK_PWM`：启动脉冲 PWM 强度。
  - `START_KICK_MS`：启动脉冲持续时间。

- `ESP32/car_ctrl_esp32_gmr/car_ctrl_esp32_gmr.ino`
  - `ENCODER_PULSES_PER_MOTOR_REV = 500`：GMR 编码器线数配置。
  - `GEAR_RATIO = 48.0f`：当前 1:45 标称 TT 马达按实际约 1:48 处理。
  - `ENCODER_SIGN`：修正编码器测速方向。

- `python/src/actions/base_action.py`
  - `motor_rating` 用来调整四个轮子的速度比例，建议使用正比例值，例如 `[1, 1, 1, 1]` 或 `[0.5, 0.5, 0.5, 0.5]`。
  - 电机方向建议在 ESP32 端通过 `MOTOR_SIGN` 修正，不在 Atlas 端用负比例修正。

## 常见问题

- `import serial` 后没有 `serial.Serial`：卸载错误的 `serial` 包，重新安装 `pyserial`。

```bash
python3 -m pip uninstall -y serial
python3 -m pip install --force-reinstall pyserial==3.5
```

- 找不到 ESP32 串口：先运行 `python3 motion_test.py --list-ports`，通常设备是 `/dev/ttyUSB0`。
- 低速下轮子不启动：适当增大 `START_KICK_PWM` 或 `START_KICK_MS`。
- 启动太猛：适当减小 `START_KICK_PWM` 或 `START_KICK_MS`。
- 只有 USB 供电时电机不转：电机需要独立电源，USB 只负责 ESP32 通信和供电。
- 普通 PC 无法直接运行完整 `python/main.py`：`acl`、`ais_bench`、`torch`、`torchvision` 等推理组件依赖 Atlas/CANN 环境。
