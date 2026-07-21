# Atlas200I DKA2 智能小车项目

本目录是项目交付版本。小车由 Atlas200I DKA2、ESP32 IOT & Robot Board、四个 GMR 编码器 TT 马达和双舵机摄像头云台组成。Atlas 通过 USB 串口发送 14 字节控制帧，ESP32 负责 PWM 电机驱动和云台控制。

## 选择运行方式

| 实现 | 目录 | 入口 | 适用场景 |
| --- | --- | --- | --- |
| 基础版 | `base/` | `main.py` | 不使用 ROS2，直接运行原始多进程控制链路 |
| ROS2 版 | `ros2/` | `ros2_ws/.../full_stack.launch.py` | 课程要求 ROS2 节点、话题和服务时使用 |

两种方式使用相同的 ESP32 串口协议、模型权重和运动参数，但运行时目录彼此独立。一次只启动其中一种，避免两套程序同时占用 `/dev/ttyUSB0` 和摄像头。

## 目录

```text
Car22/
├─ ESP32/               # Arduino 固件、接线测试与工具
├─ base/                # 非 ROS 完整实现，含 weights/
├─ ros2/                # ROS2 Humble 完整实现，含 ROS2 包与权重
├─ car_model_training/  # LaneNet / YOLO 训练、评估、ONNX 和 ATC 转换工具
└─ docs/                # 部署与使用说明
```

## 最短流程

1. 按 [ESP32/README.md](ESP32/README.md) 使用 Arduino IDE 烧录 `ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino`。
2. 首次测试务必架空小车，确认四轮方向和云台位置 `[93, 162]`。
3. 将本目录复制到 Atlas；选择 `base/README.md` 或 `ros2/README.md` 完成环境配置和启动。
4. 上位机均监听 Atlas 的 `127.0.0.1:8080`。在电脑上建立隧道：

```bash
ssh -N -L 8080:127.0.0.1:8080 root@小车IP
```

浏览器打开 `http://127.0.0.1:8080/`。

## 关键约定

- 推荐固件是纯 PWM 版本；GMR PID 版本仅作编码器闭环研究参考。
- 所有默认云台指令均为 Pan `93`、Tilt `162`。
- 部署权重已随两套实现保留：`base/weights/*.om` 与 `ros2/ros2_ws/src/smart_car_nodes/weights/*.om`。
- 训练模块不参与车端运行；训练或模型转换请查看其目录内说明。

详细部署步骤见 [docs/deployment.md](docs/deployment.md)。
