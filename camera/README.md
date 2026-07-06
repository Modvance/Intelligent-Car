# 摄像头画面调试

`camera_debug_stream.py` 用于在小车上开启一个 HTTP 摄像头画面服务。本机通过 SSH 端口转发后，可以直接在浏览器里看到小车摄像头画面，方便手动调整摄像头角度。

## 在小车上启动视频流

从 `Car22` 目录运行：

```bash
python3 camera/camera_debug_stream.py --camera 0 --max-camera-index 5 --width 1280 --height 720 --fps 30 --port 8080 --crosshair
```

或者进入 `Car22/camera` 后运行：

```bash
python3 camera_debug_stream.py --camera 0 --max-camera-index 5 --width 1280 --height 720 --fps 30 --port 8080 --crosshair
```

启动后终端会打印 SSH 转发提示。脚本默认只监听 `127.0.0.1`，适合通过 SSH tunnel 从本机访问。

## 在本机建立 SSH 端口转发

本机另开一个终端：

```bash
ssh -L 8080:127.0.0.1:8080 root@小车IP
```

如果不是用 `root` 登录，把 `root@小车IP` 换成实际 SSH 用户和 IP。

## 在本机浏览器打开画面

```text
http://127.0.0.1:8080
```

可用入口：

- `http://127.0.0.1:8080/`：调试页面。
- `http://127.0.0.1:8080/stream.mjpg`：直接查看 MJPEG 视频流。
- `http://127.0.0.1:8080/snapshot.jpg`：获取当前帧截图。
- `http://127.0.0.1:8080/health`：查看服务状态。

## 常用参数

- `--camera 0`：首选 OpenCV 摄像头编号。
- `--max-camera-index 5`：如果首选编号不可用，继续扫描 `0..5` 中第一个可读摄像头。
- `--width 1280 --height 720`：采集分辨率。
- `--fps 30`：目标帧率。
- `--quality 80`：JPEG 压缩质量，范围 `1..100`。
- `--port 8080`：小车端 HTTP 服务端口。
- `--crosshair`：显示中心十字线，方便调整摄像头角度。
- `--timestamp`：显示时间戳。
- `--flip 0`：上下翻转。
- `--flip 1`：左右翻转。
- `--flip 2`：上下和左右同时翻转。
- `--ssh-target root@小车IP`：只影响终端打印的 SSH 提示。

## 常见问题

- 页面一直等待：确认小车端脚本还在运行，且浏览器访问的是本机 `127.0.0.1:8080`。
- 提示无法打开摄像头：检查 `/dev/video*`，必要时增大 `--max-camera-index`。
- 本机打不开页面：确认 SSH 转发命令没有退出。
- 画面方向不对：使用 `--flip 0/1/2` 调整。
- 占用端口：把小车端脚本和 SSH 转发命令里的 `8080` 同时换成另一个端口，例如 `8081`。
