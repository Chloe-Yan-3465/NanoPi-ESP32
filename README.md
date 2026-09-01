# NanoPi + ESP32 + RealSense Mode 2 同步采集系统

本仓库当前版本更新于 2026-09-01，用于三套可穿戴 RGBD 采集设备：头部、左手和右手各由一台 Intel RealSense D435i、一块 NanoPi NEO3 Plus 和一块 wearable ESP32-S3 组成，另有一块 ESP32-S3 作为无线同步中控。

仓库只包含 NanoPi 和 ESP32 代码。Windows 端 `BLE-TimeSync` 控制程序独立运行，通过 USB 串口控制中控 ESP32。

## 2026-09-01 更新

- Windows START 命令可携带 `TASK` 与 `LEVEL`，采集数据按任务、难度和 episode 分层保存。
- wearable ESP32 将任务元数据随 START 转发给 NanoPi；裸 `START` 继续兼容，默认使用 `test/L_test`。
- Depth 默认采用无损 `RVL + Zstd level 1` 编码并保存为 `.rvz`，降低持续采集的磁盘占用。
- NanoPi 只保留 `nanopi/autostart/` 一套开机自启动方案，增加 RealSense USB3、帧率和 UART 运行状态检查及故障恢复。

## 目录

```text
.
├── README.md
├── nanopi/
│   ├── README.md
│   ├── CMakeLists.txt
│   ├── build.sh
│   ├── start_tmux.sh
│   ├── stop_tmux.sh
│   ├── autostart/
│   │   ├── install_autostart.sh
│   │   ├── start_tmux_autostart.sh
│   │   ├── recover_realsense_usb.sh
│   │   ├── nanopi-capture.service
│   │   └── deploy_to_nanopi.sh
│   ├── camera_cap/
│   │   ├── mode2_capture.cpp
│   │   ├── uart_camera_receiver.cpp
│   │   ├── camera_control_shm.h
│   │   ├── depth_codec.h
│   │   └── config.yaml
│   ├── common/
│   │   └── camera_control_shm.h
│   └── tools/
│       └── parse_mode2_index.py
└── esp32/
    ├── README.md
    ├── PROTOCOL.md
    ├── esp32_mode2_sync_timesync.cpp
    └── platformio_timesync.ini
```

## 系统角色

| 角色 | 数量 | 作用 |
|---|---:|---|
| Windows 控制端 | 1 | UTC 授时、按键 START/STOP、记录控制日志 |
| Coordinator ESP32 | 1 | USB 串口连接 Windows，BLE 管理三个 wearable 节点 |
| Wearable ESP32 | 3 | 接收共同时间计划，向 D435i 输出 30 Hz 脉冲，通过 UART 控制 NanoPi |
| NanoPi NEO3 Plus | 3 | 采集、MPP MJPEG 编码、分块写盘、保存二进制索引 |
| RealSense D435i | 3 | Mode 2 外部从机；输出 RGB YUYV 与 Depth Z16 |

`connected=1` 是单个节点的布尔连接状态。三节点正常时会分别出现 `node=1/2/3 connected=1`；成功授时集中显示 `TIME_ACCEPT ... nodes=3`。

## 控制与数据流

```text
Windows BLE-TimeSync
  │ USB CDC 115200：TIME_QUERY / TIME_SET / START TASK=... LEVEL=... / STOP
  ▼
Coordinator ESP32
  │ BLE：ping/pong 时钟模型、授时计划、录制计划
  ├──────────────────┬──────────────────┐
  ▼                  ▼                  ▼
Wearable node 1    Wearable node 2    Wearable node 3
  │                  │                  │
  │ GPIO2 30 Hz      │ GPIO2 30 Hz      │ GPIO2 30 Hz
  ▼                  ▼                  ▼
D435i head          D435i left         D435i right
  │ USB RGBD          │ USB RGBD         │ USB RGBD
  ▼                  ▼                  ▼
NanoPi head         NanoPi left        NanoPi right
  ▲                  ▲                  ▲
  │ UART 115200       │ UART 115200      │ UART 115200
  └── TIMESYNC / START / STOP / ACK ────┘
```

### 授时

Windows 使用 NTP 风格串口交换建立 Windows UTC 与中控微秒时钟的映射。中控再把一个未来共同时间点分发给三个 wearable，wearable 在该时间点附近通过 UART 发送 `TIMESYNC+...` 给 NanoPi。

NanoPi UART receiver 默认使用 `idle` 模式：

- 没有 episode 录制时，允许更新 `CLOCK_REALTIME`。
- 相机处于 `STARTING/RUNNING/STOPPING` 时，只记录授时帧，不修改系统时间。

### 开始录制

Windows 程序显示 `[READY]` 后直接按 `1`，无需 Enter：

1. Windows 先执行一次 episode 前授时。
2. 中控生成未来共同启动时间。
3. wearable 先通过 UART 请求 NanoPi 打开写盘 session，并等待 ACK。
4. 三个节点都 READY 后，中控 ARM 所有 wearable。
5. wearable 在共同时间点启动 30 Hz RMT 脉冲，D435i Depth 以 Mode 2 外部从机方式工作。

### 停止录制

Windows 程序直接按 `0`：

1. 中控向 wearable 发送共同 STOP。
2. wearable 停止 RMT，并通过 UART 请求 NanoPi STOP。
3. NanoPi 排空编码/写盘队列，关闭分块文件，然后返回帧数 ACK。

不要用 `Ctrl+C` 代替 episode STOP。`Ctrl+C` 只用于在确认写盘关闭后退出整个 Windows 控制进程。

## 采集数据

每个相机目录包含：

```text
<task_name>/<complex_level>/ep_YYYYMMDD_HHMMSS_<session_id>/camera_name/
├── capture_config.yaml
├── manifest.yaml
├── rgb_000000.mjpg
├── depth_000000.rvz
└── index_000000.bin
```

- RGB：D435i YUYV，经 Rockchip MPP 编码为逐帧 MJPEG。
- Depth：默认将原始小端 Z16 逐帧进行无损 RVL + Zstd level 1 编码；可通过 `storage.depth_codec` 切换编码，也可关闭写盘，但 Depth stream 仍保持开启以支持 Mode 2。
- Index：每帧保存 RGB/Depth 帧号、传感器时间、RealSense 时间域、主机接收 UTC、分块文件偏移量和字节数。

`nanopi/tools/parse_mode2_index.py` 可在采集后导出 TXT/CSV/JSONL。时间戳表中的 `source_index + rgb/depth_offset + rgb/depth_bytes` 可以精确定位分块文件中的 RGBD 帧。

## 推荐启动顺序

1. 三台 NanoPi 执行 `nanopi/start_tmux.sh`，或安装 `nanopi/autostart/` 中的 systemd 服务；确认相机 READY、UART shared memory 已连接。
2. 启动三个 wearable ESP32。
3. 启动 coordinator ESP32，确认 node 1、2、3 均 `connected=1`、`state=IDLE`。
4. 启动 Windows `BLE-TimeSync`，等待 `TIME_ACCEPT ... nodes=3` 和 `[READY]`。
5. 按 `1` 开始；按 `0` 停止。

具体编译与配置请分别阅读 [NanoPi 使用说明](nanopi/README.md) 和 [ESP32 使用说明](esp32/README.md)。
