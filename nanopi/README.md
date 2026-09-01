# NanoPi 采集端

本目录是可以复制到三块 NanoPi NEO3 Plus 的部署包。相机进程负责 RealSense/MPP/分块写盘，UART receiver 负责接收 wearable ESP32 的授时与 START/STOP，并通过共享内存控制相机进程。

## 目录

```text
nanopi/
├── CMakeLists.txt
├── build.sh
├── start_tmux.sh
├── stop_tmux.sh
├── autostart/
│   ├── install_autostart.sh
│   ├── start_tmux_autostart.sh
│   ├── recover_realsense_usb.sh
│   ├── nanopi-capture.service
│   └── deploy_to_nanopi.sh
├── camera_cap/
│   ├── mode2_capture.cpp
│   ├── uart_camera_receiver.cpp
│   ├── camera_control_shm.h
│   ├── depth_codec.h
│   └── config.yaml
├── common/
│   └── camera_control_shm.h
└── tools/
    └── parse_mode2_index.py
```

两个 `camera_control_shm.h` 内容相同。保留两份是为了兼容当前两个已实测 CPP 的相对 include 路径，不需要修改源码。

## 系统要求

- Linux AArch64，已在 NanoPi NEO3 Plus 上测试。
- CMake 3.16+、支持 C++17 的 GCC。
- librealsense2 开发文件。
- yaml-cpp 开发文件。
- Rockchip MPP 开发文件与 `rockchip_mpp.pc`。
- libzstd 开发文件与 `libzstd.pc`。
- pthread、POSIX realtime/shared memory。
- `tmux`、`stdbuf`。
- 自启动脚本还需要 `flock`、`awk`、`find`、`grep`、`sed`、`stat` 和 `systemd`。

检查：

```bash
cmake --version
g++ --version
pkg-config --modversion rockchip_mpp
pkg-config --cflags --libs rockchip_mpp
pkg-config --modversion libzstd
ldconfig -p | grep -E 'librealsense2|libyaml-cpp|librockchip_mpp|libzstd'
ls -l /dev/mpp_service
```

## 复制与编译

建议复制到：

```text
/home/pi/nanopi
```

编译命令：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 1
```

生成：

```text
build/mode2_capture
build/uart_camera_receiver
```

仅编译 UART receiver 时也可以使用：

```bash
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
    camera_cap/uart_camera_receiver.cpp \
    -Icommon -pthread -lrt \
    -o build/uart_camera_receiver
```

相机程序依赖较多，推荐始终通过 CMake 编译。

## 每块板的配置

编辑：

```bash
nano camera_cap/config.yaml
```

三块板应分别配置唯一名称：

```yaml
device:
  camera_name: "head_cam"
```

```yaml
device:
  camera_name: "left_cam"
```

```yaml
device:
  camera_name: "right_cam"
```

不要把左手配置留成 `head_cam`，否则 manifest 中的相机身份会错误。

还需要检查：

```yaml
device:
  serial: ""              # 每块板只有一台 RealSense 时可以留空

sync:
  inter_cam_sync_mode: 2  # D435i external slave

storage:
  base_dir: "/home/pi/data_mode2"
  write_depth: true       # false 仅停止写 Depth，不关闭 Depth stream
  queue_capacity: 24
  chunk_frames: 300

control:
  mode: "shared_memory"
```

RGB 固定使用 1280×720@30 YUYV，由 Rockchip MPP 编码为 MJPEG；Depth 为 1280×720@30 Z16。

## UART

默认参数：

```text
设备：/dev/ttyS1
波特率：115200
格式：8N1
授时模式：idle
```

wearable ESP32 与 NanoPi 必须共地，并交叉连接 TX/RX。ESP32 固件默认使用 UART RX=GPIO41、TX=GPIO42。

UART receiver 需要设置 `CLOCK_REALTIME`，因此 `start_tmux.sh` 使用 `sudo` 启动它。首次启动可能需要在 receiver 窗口输入 NanoPi 密码。

## tmux 启动

```bash
cd /home/pi/nanopi
bash ./start_tmux.sh
```

启动顺序有硬性保证：

```text
mode2_capture
  → 创建新的 /dev/shm/yuv_ep_flag
  → 输出 [CAPTURE] ready
  → uart_camera_receiver 才启动
```

可以覆盖默认参数：

```bash
SERIAL_DEV=/dev/ttyS1 \
CONFIG_PATH=/home/pi/nanopi/camera_cap/config.yaml \
TIME_SET_MODE=idle \
bash ./start_tmux.sh
```

日志位于：

```text
logs/camera_YYYYMMDD_HHMMSS.log
logs/uart_YYYYMMDD_HHMMSS.log
```

常用 tmux 按键：

- `Ctrl+B`，再按 `D`：退出界面但保持后台运行。
- `Ctrl+B`，再按 `0/1`：切换 camera/receiver 窗口。
- `tmux attach -t mode2_capture_20260808`：重新进入。

## 开机自启动

项目只保留一套正式自启动实现，位于 `autostart/`。它会等待 RealSense
以 USB 3.x 正常枚举并稳定输出帧，再启动 UART receiver；systemd 会监控两个
tmux 窗口，在非录制状态下发现相机停帧时重启整套服务。

安装前先停止手动启动的采集，然后执行：

```bash
cd /home/pi/nanopi
sudo bash ./autostart/install_autostart.sh
```

安装器默认使用项目目录 `/home/pi/nanopi` 和运行用户 `pi`。如果部署位置或
用户名不同，必须显式传入：

```bash
sudo bash ./autostart/install_autostart.sh /absolute/project/path username
```

安装后 service 中的关键路径应为：

```ini
WorkingDirectory=/home/pi/nanopi
Environment=HOME=/home/pi
ExecStart=/home/pi/nanopi/start_tmux_autostart.sh
```

检查和诊断：

```bash
systemctl status nanopi-capture.service
journalctl -u nanopi-capture.service -b -f
tmux attach-session -t mode2_capture_20260808
```

停止或重新启动开机服务：

```bash
sudo systemctl stop nanopi-capture.service
sudo systemctl restart nanopi-capture.service
```

从另一台 Linux 电脑向已经放置好并编译好项目的 NanoPi 更新自启动文件：

```bash
bash ./autostart/deploy_to_nanopi.sh 192.168.8.68
```

该命令默认远端用户为 `pi`、项目目录为 `/home/pi/nanopi`。它只更新自启动
文件，不上传或编译采集程序。

## 正常采集与停止

Windows 控制端按 `1` 后，UART 日志应出现 START，camera 日志应出现：

```text
[WRITER] session opened: ...
[SESSION] START ...
[STATS] rec=ON ...
```

录制期间收到授时帧时应显示：

```text
[INFO] system time unchanged due to set_mode=idle camera_recording=1
```

Windows 按 `0` 后应看到：

```text
[WRITER] session closed; frames=...
[SESSION] STOP ... frames=...
[STATS] rec=OFF ...
```

`stop_tmux.sh` 用于关闭整套 NanoPi 后台服务，不是 episode STOP。应先在 Windows 按 `0` 并等待写盘关闭，再执行：

```bash
bash ./stop_tmux.sh
```

## 数据格式

```text
<task_name>/
└── <complex_level>/
    └── ep_YYYYMMDD_HHMMSS_<session_id>/
        └── camera_name/
            ├── capture_config.yaml
            ├── manifest.yaml
            ├── rgb_000000.mjpg
            ├── depth_000000.rvz
            └── index_000000.bin
```

`task_name` 与 `complex_level` 随 START 从 Windows UI 下发，只允许字母、
数字、下划线和连字符。默认兼容值为 `test/L_test`。

RGB 分块文件由连续独立 JPEG 帧组成。Depth 默认把每个 1280×720 little-endian uint16 帧以无损 RVL + Zstd level 1 编码后连续写入 `.rvz`；可通过 `storage.depth_codec` 选择其他编码。Index 每行保存 payload offset/bytes，因此每帧压缩数据都可以独立定位，不需要把采集过程改成大量小文件。

## 解析 index

脚本只使用 Python 标准库。对单个相机：

```bash
python3 tools/parse_mode2_index.py /data/ep_xxx/head_cam \
    -o /data/ep_xxx/head_cam_timestamp.txt
```

递归解析整个 episode：

```bash
python3 tools/parse_mode2_index.py /data/ep_xxx --recursive
```

输出表包含：

```text
source_index, chunk_id, record_in_chunk, sequence
rgb_frame_number, rgb_offset, rgb_bytes, rgb_rs_timestamp_ms
depth_frame_number, depth_offset, depth_bytes, depth_rs_timestamp_ms
host_receive_unix_ns, timestamp domains, flags
```

映射规则：

```text
index_000001.bin → rgb_000001.mjpg / depth_000001.rvz
```

对 RGB 执行 `seek(rgb_offset)` 后读取 `rgb_bytes`，得到一帧完整 JPEG；对 Depth 执行 `seek(depth_offset)` 后读取 `depth_bytes`，再按 `capture_config.yaml`/index header 记录的 codec 解码为 Z16。只有配置为 `raw_z16` 时，payload 才能直接按 `<u2` reshape 为 `(720, 1280)`。
