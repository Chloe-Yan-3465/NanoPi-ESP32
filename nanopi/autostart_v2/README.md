# NanoPi autostart v2

This directory is independent from `nanopi/autostart/`; v1 is intentionally
left unchanged.

v2 waits for a USB 3.x RealSense with all expected video nodes to remain
stable, then requires several real frameset-rate samples before it starts the
UART receiver. If an abnormal capture-process exit leaves an idle USB3 camera
online without video nodes, a root-owned pre-start hook re-authorizes only that
incomplete camera. It refuses to kill a live manual tmux session, watches for
a stalled camera log and persistent idle zero-FPS, and performs a graceful
receiver/camera shutdown before systemd restarts it.

The installed unit is still named `nanopi-capture.service`. This is
intentional: v2 replaces v1 instead of creating two services that can compete
for the same camera.

Before installation, stop any manually started acquisition. Then run:

```bash
cd /home/pi
sudo ./autostart_v2/install_autostart.sh /home/pi pi
```

Useful checks:

```bash
systemctl status nanopi-capture.service
journalctl -u nanopi-capture.service -f
tmux attach-session -t mode2_capture_20260808
```

From a computer with `ssh` and `scp`, deploy the same verified bundle to
another NanoPi with one command:

```bash
./deploy_to_nanopi.sh 192.168.8.68
./deploy_to_nanopi.sh 192.168.8.69
```

The script uploads this v2 directory and runs the installer remotely. It may
prompt for the NanoPi SSH password and its `sudo` password.
