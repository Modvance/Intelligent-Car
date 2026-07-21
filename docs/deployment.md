# 部署说明

## 1. 烧录 ESP32

Arduino IDE 打开 `../ESP32/car_ctrl_esp32_pwm/car_ctrl_esp32_pwm.ino`，选择 ESP32 开发板和对应串口后上传。首次测试将车架空；固件上电会停止电机并把云台设为 `[93, 162]`。

## 2. 复制项目

将整个 `Car22` 复制到 Atlas，例如：

```bash
mkdir -p /home/HwHiAiUser/E2ESamples/src/E2E-Sample
scp -r Car22 root@小车IP:/home/HwHiAiUser/E2ESamples/src/E2E-Sample/
```

之后在 Atlas 中选择一种实现：

- 非 ROS：进入 `Car22/base`，按 `base/README.md` 启动。
- ROS2：进入 `Car22/ros2`，按 `ros2/README.md` 构建并启动。

## 3. 上位机

两种实现的上位机默认都只监听 Atlas 本机。电脑上执行：

```bash
ssh -N -L 8080:127.0.0.1:8080 root@小车IP
```

然后访问 `http://127.0.0.1:8080/`。端口被占用时，将小车启动命令中的 `monitor_port` 或 `--monitor-port` 改为其他端口，并让 SSH 隧道使用相同端口。

## 4. 安全顺序

先架空测试串口和轮子方向，再测试摄像头与模型，最后放到赛道上。运行时只能启动一套实现；需要切换时先按 `Ctrl+C` 完整停止当前程序。
