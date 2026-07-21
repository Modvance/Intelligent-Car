# ROS2 小车实现

这是在不修改基础版行为的前提下迁移的 ROS2 Humble 版本。它自带 LaneNet、YOLO 推理封装和 `lanenet.om`、`yolo.om` 权重，运行时不依赖 `../base`。只启动本实现时，ROS2 是唯一的串口和摄像头所有者。

## ROS2 图

```text
camera_node -> /camera/image/compressed -> lane_node -----------+
                                      -> sign_detector_node -----+-> decision_node -> /car/command -> esp32_driver_node -> ESP32
                                                                  |                       |
                                                                  +---- monitor_node <---+-> /decision/state, /car/state
```

`decision_node` 是自动模式唯一的运动指令发布者。优先级固定为：决策关闭 > 行人停车 > 正在执行的地标动作 > 巡线。行人出现时会立即停止并冻结动作计时；连续两帧确认行人离开后，未完成的动作从暂停处继续，若没有动作则执行旧版 `Start` 启动脉冲再交回巡线。

## 已迁移行为

- ESP32 原有 14 字节串口协议和 PWM 固件接口。
- 旧版电机倍率 `0.96, 0.96, 0.8, 0.8`，自动模式舵机位置 `[93, 162]`。
- LaneNet 朝向与车道中心横向修正：`alpha=0.8`、直行 `26`、转向 `20`。
- 地标动作：前三次 `left/right` 标志执行左转，之后执行右转；掉头、泊车、停车动作时序保持旧版值。
- 可开关的“两次右转后才允许泊车”门控，默认开启。
- 指定区域的小人检测停车与恢复。
- 与 `old_car` 同结构的浏览器上位机：Camera、System、Motor Control 曲线、Lane Detection、Sign Detection、Controls、Events 和原始调试状态；页面按钮对应 ROS 的决策/运动门控服务。

## Atlas 前置检查

项目在 Atlas 的目标路径：

```bash
cd /home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22/ros2
```

ROS2 节点使用系统 Python 3.10；模型推理由节点启动 `/usr/local/miniconda3/bin/python3`（你当前已验证能运行旧版模型的 Python 3.9）子进程完成。因此先确认两套环境都可用：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -c "import rclpy; print('ROS Python OK')"
/usr/local/miniconda3/bin/python3 -c "import cv2, torch, acl; from ais_bench.infer.interface import InferSession; print('Inference Python OK')"
```

摄像头节点和浏览器叠加层需要系统 Python 有 OpenCV：

```bash
/usr/bin/python3 -c "import cv2; print(cv2.__version__)"
```

若最后一条失败，再安装系统 OpenCV：

```bash
sudo apt update
sudo apt install -y python3-opencv
```

如果 Miniconda 不在 `/usr/local/miniconda3/bin/python3`，在启动命令末尾追加：

```text
inference_python:=实际路径
```

## 构建

```bash
cd /home/HwHiAiUser/E2ESamples/src/E2E-Sample/Car22/ros2/ros2_ws
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

每次打开新终端后，重新执行上面的两条 `source`。编译后确认节点和消息存在：

```bash
ros2 pkg executables smart_car_nodes
ros2 interface show smart_car_msgs/msg/DecisionState
```

## 分阶段测试

先只开感知和上位机，车不会发送任何电机指令：

```bash
ros2 launch smart_car_bringup perception.launch.py camera:=0 monitor_port:=8080
```

本机浏览器通过 SSH 隧道查看：

```bash
ssh -L 8080:127.0.0.1:8080 root@小车IP
```

打开 `http://127.0.0.1:8080/`，确认 LaneNet 线条、YOLO 框和画面帧率正常。停止使用 `Ctrl+C`。

再把小车架空，启动完整栈。默认“模型全开、决策关闭、电机门控关闭”：

```bash
ros2 launch smart_car_bringup full_stack.launch.py port:=/dev/ttyUSB0 camera:=0 monitor_port:=8080
```

另开一个已 `source install/setup.bash` 的终端，按顺序打开两个门控：

```bash
ros2 service call /motion_gate/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /decision_gate/set_enabled std_srvs/srv/SetBool "{data: true}"
```

`decision_gate` 打开后会先执行旧版的 0.2 秒启动脉冲。确认架空测试方向、地标动作和行人停车都正确，再放到赛道跑图。

紧急停止：先关闭电机门控，它会在下一个 20 Hz 决策帧将电机输出归零；也可同时关闭决策门控：

```bash
ros2 service call /motion_gate/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /decision_gate/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 调参

统一参数文件在 `ros2_ws/src/smart_car_bringup/config/full_stack.yaml`。跑图时优先修改这里，重新 `colcon build --symlink-install` 后重启 launch。

- 巡线：`straight_speed`、`turn_speed`、`turn_trigger_deg`、`turn_gain`。
- 地标区域：`turn_region_*`、`park_region_*`、`park_score_threshold`。
- 泊车门控：`park_right_turn_gate_enabled`。设为 `false` 时只依据正常检测区域触发。
- 小人停车：`human_region_*`、`human_score_threshold`、`human_clear_frames`。

## 手动模式

手动模式只验证底层，不加载模型：

```bash
ros2 launch smart_car_bringup manual_control.launch.py port:=/dev/ttyUSB0
```

## 本机回归测试

在当前开发电脑运行：

```powershell
python -m unittest discover Car22/ros2/tests -v
python -m compileall Car22/ros2/ros2_ws/src/smart_car_nodes/smart_car_nodes
```
