# Stable Mode2 baseline — 2026-08-11

Git tag: `stable-2026-08-11-mode2`

This is the fixed NanoPi/ESP32 half of the 2026-08-11 hardware-validated
baseline. The matching `DataCapture_UI` repository uses the same tag name.

## Required combination

- ESP32 source: `esp32/esp32_mode2_sync_timesync.cpp`
- ESP32 environments: `esp32/platformio_timesync.ini`
- NanoPi camera: `nanopi/camera_cap/mode2_capture.cpp`
- NanoPi UART bridge: `nanopi/camera_cap/uart_camera_receiver.cpp`
- NanoPi launch script: `nanopi/start_tmux.sh`

Do not replace only one item with an older copy. The STOP frame report depends
on the complete path:

```text
NanoPi ACK+STOP+FRAMES
  -> wearable status
  -> coordinator STOP_FRAME
  -> Windows UI
```

## Stabilized behavior

- Wearable BLE notifications run through their own queue/task.
- Continuous PING uses write-without-response; control commands retain
  acknowledged writes.
- Coordinator scan completion and wearable connection run asynchronously.
- Scanning is explicit (`SCAN`) and does not interrupt already connected nodes.
- NanoPi START/STOP ACKs include the closed episode frame count.
- Existing RGB status-light effects remain enabled.

## Hardware validation

The coordinator, head, and right were run for 1800 seconds with eight RGBD
episodes of unequal length and unequal idle intervals.

- Head frames: 1074, 2364, 1468, 2727, 985, 2029, 1678, 2539.
- Right frames: 1077, 2366, 1466, 2726, 984, 2033, 1677, 2542.
- Head camera: no sample below 20 Hz, no 0 Hz, no queue/oversize drops.
- Right camera: formal-episode minimum 25.9 Hz, no 0 Hz, no queue/oversize drops.
- UART: all formal START/STOP commands returned `OK` and all STOP frame counts
  matched the coordinator reports.
- BLE: no disconnect after the formal connection point and no non-zero node
  error flags.

The first partial right-camera startup window was 19.9 Hz; all following idle
samples stabilized at approximately 30 Hz. This is not a sustained drop.

## Rebuild check

From `esp32/`:

```powershell
platformio run -c platformio_timesync.ini \
  -e coordinator_timesync -e coordinator_single_timesync \
  -e wearable_head_timesync -e wearable_left_timesync \
  -e wearable_right_timesync
```

All five environments passed before this baseline was tagged.

## Operational note

A physically unplugged or intermittent RealSense USB connection reproduces the
observed `30 Hz -> teens -> 0 Hz` signature. Treat that signature first as a
cable, connector, power, or USB-link fault. The validated repeated START/STOP
sequence did not reproduce it.
