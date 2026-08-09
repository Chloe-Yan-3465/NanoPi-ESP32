# Mode 2 + Windows UTC 授时协议

本文对应：

- `esp32_mode2_sync_timesync.cpp`
- `platformio_timesync.ini`

原版 `esp32_mode2_sync.cpp` 和 `platformio.ini` 不参与此版本构建，也没有修改。

## 1. 链路

```text
Windows UTC/QPC
    |
    | USB CDC 串口，115200
    v
Mode2Coordinator
    |
    | 已有 BLE 连接和 ping/pong 时钟模型
    v
Mode2Node-1 / Mode2Node-2 / Mode2Node-3
    |
    | UART，115200 8N1，ESP32 RX=GPIO41，TX=GPIO42
    v
NanoPi uart_camera_receiver
```

Mode 2 的相机触发协议、BLE UUID、60字节 `WireMessage` 布局、START/STOP状态机均保持不变。授时只新增 `MessageType::time_sync = 8`。

## 2. Windows到中控ESP32

全部命令都是ASCII单行，以 `\n` 结束；中控同时接受 `\r\n`。

### 2.1 简单授时

```text
TIME <unix_sec> <usec>\n
```

示例：

```text
TIME 1786176000 123456
```

- `unix_sec`：UTC Unix秒，允许范围为2020-01-01至2100-01-01。
- `usec`：0到999999。
- 时间值应尽可能靠近Windows真正写出该行的时刻生成。
- 中控将收到该行的时刻视为UTC参考点。
- 该入口没有USB往返时延校正，固件把基础不确定度标为20000微秒，适合连通性测试。

中控接受后输出：

```text
TIME_PLAN seq=<seq> target_coordinator_us=<mono_us> utc_ns=<unix_ns>
```

完成向全部节点下发后输出：

```text
TIME_ACCEPT seq=<seq> nodes=<count> uncertainty_us=<us>
```

`TIME_ACCEPT`表示三个slave已接收发送计划，不表示NanoPi已经完成 `clock_settime`。

Windows控制程序必须等待本轮 `TIME_ACCEPT` 后再发送 `START`。如果中控已经进入START的READY/ARM关键窗口，授时分发会暂停；错过未来锚点时该轮授时会返回 `TIME_ERROR`，Windows应重新授时，不能把该轮视为成功。

### 2.2 双向测时接口（推荐）

Windows发送：

```text
TIME_QUERY <seq>\n
```

中控立即返回：

```text
TIME_REPLY <seq> <coordinator_rx_us> <coordinator_tx_us>\n
```

Windows应记录：

- `t1`：写出 `TIME_QUERY` 前的Windows UTC纳秒。
- `t4`：完整收到 `TIME_REPLY` 后的Windows UTC纳秒。
- `t2`：`coordinator_rx_us`。
- `t3`：`coordinator_tx_us`。

建议连续获取多组样本，选择往返时间最小的一组。可用以下近似锚点：

```text
coordinator_ref_us = (t2 + t3) / 2
utc_ref_ns         = (t1 + t4) / 2
uncertainty_us     = max(0, ((t4 - t1) / 1000 - (t3 - t2)) / 2)
```

然后Windows发送：

```text
TIME_SET <seq> <coordinator_ref_us> <utc_ref_ns> <uncertainty_us>\n
```

限制：

- `seq`必须大于0。
- `coordinator_ref_us`必须在中控当前单调时间前后30秒以内。
- `uncertainty_us`不得大于1000000。
- `utc_ref_ns`必须位于2020至2100年范围。

Windows端应优先用QPC维持单调时间，再用 `GetSystemTimePreciseAsFileTime` 建立QPC到UTC的映射，避免直接连续调用低分辨率墙钟。

### 2.3 错误返回

错误以单行文本返回，例如：

```text
TIME_ERROR nodes are not synchronized; retry later
TIME_ERROR invalid UTC clock map
TIME_ERROR seq=<seq> missed distribution deadline
TIME_ERROR seq=<seq> node=<id> BLE write failed
```

Windows收到错误后应等待节点重新达到同步状态，再重新执行测时和 `TIME_SET`。

## 3. 中控到slave的BLE内部报文

BLE服务和特征不变：

```text
Service: 5d6f0001-4f3c-4f59-a9f2-36f36a7c1000
Command: 5d6f0002-4f3c-4f59-a9f2-36f36a7c1000 (WRITE)
Status : 5d6f0003-4f3c-4f59-a9f2-36f36a7c1000 (READ/NOTIFY)
```

`time_sync`仍使用原来的60字节小端 `WireMessage`：

| 字段 | 含义 |
|---|---|
| `type` | 8 (`time_sync`) |
| `sequence` | Windows授时序号 |
| `a` | 该slave的未来本地 `esp_timer_get_time()` 锚点，微秒 |
| `b` | 锚点对应的Windows UTC，Unix纳秒 |
| `x` | Windows/USB不确定度加该BLE链路半RTT，微秒 |
| `crc32` | 沿用现有CRC32 |

中控使用已有 `coordinatorToLocal()` 把一个共同的中控未来时刻转换到三个slave的本地单调时钟。

## 4. slave到NanoPi的UART协议

### 4.1 原有控制帧：完全不变

```text
CMD+START+SESSION=<id>+END\r\n
CMD+STOP+SESSION=<id>+END\r\n
```

NanoPi原有ACK格式也不变：

```text
ACK+START+SESSION=<id>+OK+END\r\n
ACK+START+SESSION=<id>+ERROR+...+END\r\n
ACK+STOP+SESSION=<id>+OK+END\r\n
ACK+STOP+SESSION=<id>+ERROR+...+END\r\n
```

### 4.2 授时帧：沿用旧 `v2_30hz` 格式

```text
TIMESYNC+YYYY-MM-DDTHH:MM:SS.ffffffZ+END\r\n
```

示例：

```text
TIMESYNC+2026-08-08T15:26:43.123456Z+END\r\n
```

- 固定UTC，末尾为 `Z`。
- 小数点后固定6位微秒。
- 与现有 `uart_camera_receiver.cpp` 的解析格式一致，不需要修改接收协议。
- slave在真正准备写UART时根据单调时钟重新外推UTC，控制消息导致的排队不会让时间值停留在旧计划时刻。
- 固件加入4500微秒补偿，用于近似覆盖42字节授时帧在115200波特率下的线传输和接收唤醒时间。

## 5. 并行与优先级

穿戴ESP32内部：

| 通道 | 实现 | 优先级/行为 |
|---|---|---|
| 相机脉冲 | RMT + `camera_trigger`任务 | Core 1，优先级10，保持原逻辑 |
| START/STOP UART | 长度8的控制队列 | UART最高优先级 |
| 授时计划 | 可覆盖的长度1队列 | 新样本覆盖未处理旧样本 |
| 授时UART | 可覆盖的长度1低优先级队列 | 不占用控制队列 |
| UART物理发送 | 唯一 `neo_uart_tx`任务 | 防止两帧字节交叉，不调用 `Serial1.flush()` |

UART是单一物理线路，无法同时发送两帧；这里的“并行”是指RMT、BLE、控制生产者和授时生产者互不阻塞。若授时帧已经开始传输，后到的控制帧最坏等待约4毫秒；其余情况下控制队列始终优先。

中控每次循环最多向一个slave发送一条授时BLE报文，并使用无响应写入；START正在等待READY/ARM时暂停授时下发，避免争用关键控制窗口。

## 6. NanoPi校时注意事项

当前 `uart_camera_receiver.cpp` 已支持：

```text
TIMESYNC+YYYY-MM-DDTHH:MM:SS.ffffffZ+END
```

它的 `set_mode` 为：

- `idle`：仅在相机共享内存状态不是 `STARTING/RUNNING/STOPPING` 时调用
  `clock_settime`，这是当前推荐和默认模式。
- `every`：每个授时帧都调用 `clock_settime`。
- `once`：只在第一次授时时设置系统时间。
- `none`：只认可同步状态，不修改系统时间。

Windows 每次 START 前会主动完成一轮新授时，并等待当前1.5秒授时计划经过后再发送 START。录制期间仍可继续测时和接收授时帧，但 `idle` 模式不会修改 `CLOCK_REALTIME`；STOP完成并回到IDLE后才重新允许硬校时。

## 7. PlatformIO构建

使用独立配置文件：

```bash
platformio run -c platformio_timesync.ini -e coordinator_timesync
platformio run -c platformio_timesync.ini -e coordinator_single_timesync
platformio run -c platformio_timesync.ini -e wearable_head_timesync
platformio run -c platformio_timesync.ini -e wearable_left_timesync
platformio run -c platformio_timesync.ini -e wearable_right_timesync
```

烧录时在对应命令后添加 `-t upload`。
