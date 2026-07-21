# 基础版：非 ROS 小车控制

本目录是当前已验证的非 ROS 实现。它使用多进程运行摄像头、LaneNet、YOLO、控制器和浏览器上位机；不依赖 ROS2。

## 运行前提

- Atlas 已安装 CANN/ACL、`ais_bench` 和能够加载 `.om` 的 Miniconda Python 环境。
- ESP32 已烧录 `../ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino`，并通过 USB 显示为 `/dev/ttyUSB0`。
- `weights/lanenet.om` 与 `weights/yolo.om` 已随本目录保留。

安装 Python 依赖：

```bash
cd /home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22/base
python3 -m pip install -r requirements.txt
```

若 CANN 推理依赖已由系统镜像预装，不要用普通 PC 环境覆盖其 `acl`、`ais_bench` 或昇腾相关包。

## 启动

先架空小车，确认串口：

```bash
python3 -c "from src.utils.init_utils import get_port; from src.utils.constant import ESP32_NAME; print(get_port(ESP32_NAME))"
```

手动控制：

```bash
python3 main.py --mode manual
```

自动模式。模型和摄像头会启动，但决策与运动默认关闭：

```bash
python3 main.py --mode easy --monitor --monitor-port 8080
```

`easy` 模式按键：`d` 开关决策，`g` 同时启动决策和运动，`space` 停车并关闭运动，`esc` 退出。

## 常用功能

手动模式按键：`w/s` 前进后退，`a/d` 左右转，`q/e` 逆时针/顺时针旋转，`j/l` 左右平移，`space` 停车。

手动调试地标动作：`1` 左转、`2` 右转、`3` 掉头进入段、`4` 掉头结束段、`5` 泊车、`6` 停车标志。

手动采集数据：

```bash
python3 main.py --mode manual --auto-capture --capture-interval 0.5 --capture-dir capture/manual_auto
```

## 模块职责

- `main.py`：进程启动、键盘控制和模式选择。
- `src/utils/controller.py`：唯一串口控制器；向 ESP32 发送 7 个 `int16` 组成的 14 字节控制帧。
- `src/scenes/LANENET.py`：车道线推理与巡线控制。
- `src/scenes/helper.py`：YOLO 地标/小人检测与动作触发。
- `src/actions/sign_actions.py`：手动测试和地标决策共用的动作时序。
- `src/utils/monitor_server.py`：浏览器上位机与视频流。

## 电机状态显示

上位机中的 **Shadow PID** 仅根据发送给 ESP32 的命令估算轮速趋势，用于观察
open-loop PWM 的控制状态；它不参与底层控制，也不替代编码器实测数据。

## 停止和排错

使用 `esc` 或 `Ctrl+C` 退出；控制器会发送电机零 PWM 并保持云台标定位置 `[93, 162]`。

- 找不到串口：检查 `ls /dev/ttyUSB*`，通常为 `/dev/ttyUSB0`。
- 浏览器无法访问：先确认程序日志显示监听 `127.0.0.1:8080`，再建立 SSH 隧道。
- `serial.Serial` 不存在：卸载错误的 `serial` 包并安装 `pyserial==3.5`。
- SSH 下出现 Qt/xcb 报错：不要启用本地图形窗口，使用 `--monitor` 查看画面。
