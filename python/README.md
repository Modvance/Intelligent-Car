# Atlas Python 端程序

本目录是在 Atlas200I DKA2 开发板上运行的小车 Python 程序，负责通过 USB 串口向 ESP32 发送运动控制命令，并运行摄像头、手动控制、数据采集、上位机监测、目标识别和 Lanenet 巡线逻辑。

底层 ESP32 推荐配合：

```text
../ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino
```

## 文件说明

```text
python/
├─ main.py                 # 小车主入口：manual/cmd/easy 模式
├─ motion_test.py          # 最小运动链路测试脚本
├─ camera.py               # 可选独立定时采图脚本（不控制小车、不打开网页）
├─ camera/                 # SSH 摄像头画面调试工具
├─ requirements.txt        # Atlas Python 依赖
├─ weights/                # yolo.om、lanenet.om 模型权重
└─ src/
   ├─ actions/             # 基础动作、复合动作、地标动作序列
   ├─ models/              # Atlas 离线模型推理封装
   ├─ scenes/              # 手动控制、Lanenet 巡线、目标识别等场景
   └─ utils/               # 串口控制、相机、监测服务、采图、运动闸门等工具
```

## 运行前准备

1. ESP32 已烧录：

```text
Car22/ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino
```

2. ESP32 通过 USB 连接到 Atlas 开发板。

3. 电机接好外部电源。USB 只能负责 ESP32 通信和供电，不能单独带动电机。

4. 模型权重位于：

```text
python/weights/yolo.om
python/weights/lanenet.om
```

5. 第一次运动测试时，小车必须架空。

## 安装依赖

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

查看 Atlas 是否识别到 ESP32 串口：

```bash
python3 motion_test.py --list-ports
```

架空小车后运行基础动作序列：

```bash
python3 motion_test.py --mode sequence --speed 25 --duration 0.8
```

如果自动识别串口失败，可以手动指定：

```bash
python3 motion_test.py --port /dev/ttyUSB0 --mode sequence --speed 25 --duration 0.8
```

看到 ESP32 返回 `SUCC`，说明 Atlas 到 ESP32 的串口协议链路正常。

## 手动控制

```bash
python3 main.py --mode manual
```

常用键位：

```text
w       前进
s       后退
a       左转
d       右转
q       逆时针旋转
e       顺时针旋转
j/l     左/右平移
u/p     左/右斜向运动
space   停止
c       保存当前一帧
v       开启/关闭自动采集
esc     退出
```

地标动作调试键：

```text
1       模拟左转地标动作
2       模拟右转地标动作
3       模拟第一次掉头/sideway 动作
4       模拟第二次掉头/结束段动作
5       模拟泊车动作
6       模拟停车标志动作
```

这些快捷键调用 `src/actions/sign_actions.py`，目标检测识别到标志后也会调用同一套动作序列。

## 手动采集数据集

一边手动控制小车，一边定时保存摄像头画面：

```bash
python3 main.py --mode manual --auto-capture --capture-interval 0.5 --capture-dir capture/manual_auto
```

自动采集模式会默认启动浏览器预览。本机另开终端做 SSH 端口转发：

```bash
ssh -L 8080:127.0.0.1:8080 root@小车IP
```

浏览器打开：

```text
http://127.0.0.1:8080
```

常用参数：

```text
--capture-interval 0.5      每 0.5 秒保存一张
--capture-dir capture/...   图片保存目录
--monitor-port 8080         预览页面端口
--no-capture-preview        只采图，不启动浏览器预览
--preview-overlay           手动采集预览里也叠加模型输出
```

采集完成后从小车拷回电脑：

```bash
scp -r root@小车IP:/home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22/python/capture ./capture
```

## 独立摄像头工具

如果只想看摄像头画面、调整摄像头角度，使用浏览器调试工具：

```bash
cd /home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22/python
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

`camera.py` 是一个更简单的独立定时采图脚本，会打开摄像头并每隔固定时间保存图片到 `capture/round4/`。它不会控制小车，也没有网页预览；正常采集数据集时更推荐使用 `main.py --mode manual --auto-capture`。

## 上位机监测

手动模式监测：

```bash
python3 main.py --mode manual --monitor --monitor-port 8080
```

自动驾驶模式监测：

```bash
python3 main.py --mode easy --monitor --monitor-port 8080
```

本机通过 SSH 转发访问：

```bash
ssh -L 8080:127.0.0.1:8080 root@小车IP
```

浏览器打开：

```text
http://127.0.0.1:8080
```

页面会显示摄像头画面、当前运动命令、ESP32 返回值、目标检测结果和巡线结果。手动采集模式默认显示原始摄像头画面，不叠加模型线。

## 自动驾驶 easy 模式

`easy` 模式启动 Lanenet 巡线和目标检测：

```bash
python3 main.py --mode easy --monitor --monitor-port 8080
```

默认运动闸门关闭，小车不会移动。确认画面和模型输出正常后，在终端按：

```text
g       允许运动
space   停车并重新锁住运动
esc     退出
```

如果确认环境安全，可以启动后直接允许运动：

```bash
python3 main.py --mode easy --monitor --start-motion-enabled
```

## 摄像头自动检索

主程序启动摄像头时，会优先尝试 `src/utils/constant.py` 中 `CAMERA_INFO` 配置的摄像头编号：

```python
CAMERA_INFO = {
    'height': 720,
    'width': 1280,
    'fps': 30,
    'camera': 0,
    'max_camera_index': 5
}
```

如果 `camera` 指定的编号不可用，程序会继续扫描 `0..max_camera_index`，使用第一个能成功打开并读取画面的摄像头。

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
- 摄像头打不开：检查 `/dev/video*`，必要时增大 `CAMERA_INFO['max_camera_index']`。
- `easy` 模式不动：默认运动闸门关闭，按 `g` 后才允许运动。
- 普通 PC 无法直接运行完整 `main.py`：视觉推理相关代码依赖 Atlas/CANN 环境。
- `torchvision` 的图片扩展 warning 如果不影响程序启动，可以暂时忽略。
