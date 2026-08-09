# ESP32 Mode 2 同步与授时固件

同一份 `esp32_mode2_sync_timesync.cpp` 通过 PlatformIO build flags 编译成 coordinator 或 wearable。当前版本同时包含：

- 三 wearable BLE 管理。
- coordinator↔wearable ping/pong 时钟模型。
- 未来共同时间点授时。
- 无需用户输入 session ID 的 START/STOP。
- wearable 到 NanoPi 的 UART 控制和 ACK。
- GPIO2/RMT 生成 30 Hz RealSense Mode 2 外部脉冲。

详细线协议见 [PROTOCOL.md](PROTOCOL.md)。

## 硬件角色

### Coordinator

- 通过 USB CDC 115200 连接 Windows `BLE-TimeSync`。
- BLE central，扫描并连接 `Mode2Node-1/2/3`。
- 不直接连接 NanoPi 或相机。

### Wearable

- BLE peripheral，名称分别为 `Mode2Node-1/2/3`。
- GPIO2 输出相机同步脉冲。
- UART RX=GPIO41、TX=GPIO42，115200 8N1，连接对应 NanoPi。
- 相机同步线和 UART 都必须共地。

RealSense 9-pin 同步口应采用已经验证过的 pin 5 SYNC 与 pin 9 GND 接线。接线前仍应依据相机与开发板电气规格确认信号电平；不要把电源引脚接入同步线。

## PlatformIO 环境

| 环境 | 用途 |
|---|---|
| `coordinator_timesync` | 正式三节点中控，要求 node 1/2/3 全部在线 |
| `coordinator_single_timesync` | 单 head 节点台架测试 |
| `wearable_head_timesync` | node 1，头部 |
| `wearable_left_timesync` | node 2，左手 |
| `wearable_right_timesync` | node 3，右手 |

`default_envs` 是 head wearable。为避免烧错角色，所有正式编译和烧录都应显式指定 `-e`。

## 编译

在本目录执行：

```powershell
platformio run -c platformio_timesync.ini -e coordinator_timesync
platformio run -c platformio_timesync.ini -e wearable_head_timesync
platformio run -c platformio_timesync.ini -e wearable_left_timesync
platformio run -c platformio_timesync.ini -e wearable_right_timesync
```

单节点测试：

```powershell
platformio run -c platformio_timesync.ini `
  -e coordinator_single_timesync `
  -e wearable_head_timesync
```

## 烧录

为每块 ESP32 指定它实际枚举出的 COM 口：

```powershell
platformio run -c platformio_timesync.ini `
  -e coordinator_timesync `
  -t upload --upload-port COM14
```

```powershell
platformio run -c platformio_timesync.ini `
  -e wearable_head_timesync `
  -t upload --upload-port COM15
```

上面的 `COM14`、`COM15` 只是示例，烧录时替换成对应 ESP32 当前实际使用的端口。

左手和右手分别使用：

```text
wearable_left_timesync
wearable_right_timesync
```

烧录后串口监视器使用 115200 baud。

## 启动检查

三节点中控启动时应显示：

```text
Mode2+Time coordinator ready; discovering nodes 1..3
```

三个 wearable 分别显示：

```text
Mode2+Time wearable node 1 ready
Mode2+Time wearable node 2 ready
Mode2+Time wearable node 3 ready
```

中控每个节点必须满足：

```text
connected=1
state=IDLE
samples>=6
rtt<=50000us
fresh<2500ms
error=0x00000000
```

成功授时应显示：

```text
TIME_ACCEPT seq=... nodes=3 uncertainty_us=...
utc_map=LOCKED ...
```

## Windows 控制

Windows 程序独占 coordinator 的 USB 串口，因此运行期间不要再用串口助手打开同一 COM 口。

程序显示 `[READY]` 后：

- 直接按 `1`：episode 前授时并 START，不按 Enter。
- 直接按 `0`：STOP，不按 Enter。
- `Ctrl+C`：退出控制程序；只能在确认所有 NanoPi 已停止写盘后使用。

START 成功时应显示：

```text
session ... planned; common epoch=...
session ... ARMED on all nodes
```

STOP 成功时应显示：

```text
STOP scheduled at coordinator=...
```

内部 session 数字用于协议去重和 ACK 对应，由固件自动生成；用户操作不需要提供 ID。

## NanoPi UART 协议

wearable→NanoPi：

```text
TIMESYNC+YYYY-MM-DDTHH:MM:SS.ffffffZ+END\r\n
CMD+START+SESSION=<internal>+END\r\n
CMD+STOP+SESSION=<internal>+END\r\n
```

NanoPi→wearable：

```text
ACK+START+SESSION=<internal>+OK+END
ACK+STOP+SESSION=<internal>+OK+FRAMES=<count>+END
```

授时与 START/STOP 使用独立队列和 FreeRTOS task，避免 wearable UART 写入相互阻塞。

## 单节点与三节点

单节点台架测试必须烧录 `coordinator_single_timesync`。正式三节点测试烧录 `coordinator_timesync`；三节点版本会要求 node 1、2、3 全部连接且同步就绪，缺少任何节点都会拒绝授时和 START。

不要用只启动 head 的方式测试三节点 coordinator，否则中控会持续扫描缺失节点，授时和串口响应可能出现长延迟。
