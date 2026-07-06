# Atlas Python 端程序

本目录是在 Atlas200I DKA2 开发板上运行的小车 Python 程序，负责通过 USB 串口向 ESP32 发送运动控制命令，并承载后续手动控制、巡线、目标识别等上层逻辑。

当前基础阶段建议先配合 ESP32 端的纯 PWM 版本使用：

```text
../ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino
```

## 文件说明

```text
python/
├─ main.py                 # 小车主入口，用于手动控制和后续场景运行
├─ motion_test.py          # 最小运动链路测试脚本
├─ requirements.txt        # Atlas Python 依赖
├─ image.png               # 项目附带图片资源
└─ src/
   ├─ actions/             # 基础动作和复合动作
   ├─ models/              # 模型推理封装
   ├─ scenes/              # 手动控制、巡线、目标跟踪等场景
   └─ utils/               # 串口控制、相机、ACL、日志等工具
```

## 运行前准备

1. ESP32 已经烧录当前基础运动程序：

```text
Car22/ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino
```

2. ESP32 通过 USB 连接到 Atlas 开发板。

3. 电机接好外部电源。USB 只能负责 ESP32 通信和供电，不能单独带动电机。

4. 第一次运动测试时，小车必须架空。

## 安装依赖

在 Atlas 开发板上进入本目录：

```bash
cd /home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22/python
python3 -m pip install -r requirements.txt
```

如果遇到：

```text
AttributeError: module 'serial' has no attribute 'Serial'
```

通常是误装了 `serial` 包，需要卸载后重新安装 `pyserial`：

```bash
python3 -m pip uninstall -y serial
python3 -m pip install --force-reinstall pyserial==3.5
```

## 最小运动测试

先查看 Atlas 是否识别到 ESP32 串口：

```bash
python3 motion_test.py --list-ports
```

常见串口设备是：

```text
/dev/ttyUSB0
```

架空小车后，运行基础动作序列：

```bash
python3 motion_test.py --mode sequence --speed 25 --duration 0.8
```

如果自动识别串口失败，可以手动指定：

```bash
python3 motion_test.py --port /dev/ttyUSB0 --mode sequence --speed 25 --duration 0.8
```

看到 ESP32 返回 `SUCC`，说明 Atlas 到 ESP32 的串口协议链路正常。

## 手动控制

确认 `motion_test.py` 正常后，运行主程序：

```bash
python3 main.py
```

常用键位：

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

如果程序卡住无法正常退出，先尝试 `Ctrl+C`。如果摄像头或串口被残留进程占用，结束残留 Python 进程或重新登录后再运行。

## 当前基础运动配置

- ESP32 端负责实际电机输出、方向修正和低速启动脉冲。
- Atlas 端 `src/actions/base_action.py` 中的 `motor_rating` 用于调整四个轮子的速度比例。
- `motor_rating` 建议保持正数，例如 `[1, 1, 1, 1]` 或 `[0.5, 0.5, 0.5, 0.5]`。
- 某个轮子方向反了，优先在 ESP32 端通过 `MOTOR_SIGN` 修正。
- 当前基础运动版本不依赖 ESP32 编码器 PID。

## 常见问题

- 找不到 ESP32 串口：先运行 `python3 motion_test.py --list-ports`，确认是否存在 `/dev/ttyUSB0`。
- 命令返回 `FAIL`：检查 ESP32 是否烧录了兼容 14 字节二进制协议的程序。
- 小车低速不启动：调整 ESP32 端 `START_KICK_PWM` 或 `START_KICK_MS`。
- 小车方向不对：先架空测试，再修改 ESP32 端 `MOTOR_SIGN`。
- 普通 PC 无法直接运行完整 `main.py`：视觉推理相关代码依赖 Atlas/CANN 环境。
  - `torchvision` 的图片扩展 warning 如果不影响程序启动，可以暂时忽略。