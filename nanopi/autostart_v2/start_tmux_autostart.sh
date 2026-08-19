#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SESSION_NAME="${SESSION_NAME:-mode2_capture_20260808}"
CAMERA_BIN="${CAMERA_BIN:-${ROOT_DIR}/build/mode2_capture}"
UART_BIN="${UART_BIN:-${ROOT_DIR}/build/uart_camera_receiver}"
CONFIG_PATH="${CONFIG_PATH:-${ROOT_DIR}/camera_cap/config.yaml}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyS1}"
SERIAL_BAUD="${SERIAL_BAUD:-115200}"
TIME_SET_MODE="${TIME_SET_MODE:-idle}"
SHM_PATH="${SHM_PATH:-/dev/shm/yuv_ep_flag}"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/logs}"

# USB must remain fully enumerated before librealsense is opened.
CAMERA_DEVICE_WAIT_TIMEOUT_SECONDS="${CAMERA_DEVICE_WAIT_TIMEOUT_SECONDS:-120}"
CAMERA_USB_STABLE_SECONDS="${CAMERA_USB_STABLE_SECONDS:-5}"
CAMERA_USB_PRODUCT_PATTERN="${CAMERA_USB_PRODUCT_PATTERN:-RealSense}"
CAMERA_MIN_USB_SPEED_MBPS="${CAMERA_MIN_USB_SPEED_MBPS:-5000}"
CAMERA_MIN_VIDEO_NODES="${CAMERA_MIN_VIDEO_NODES:-6}"

# [CAPTURE] ready alone only means pipeline.start() returned. Require actual
# framesets at a plausible rate before exposing UART START/STOP control.
CAMERA_READY_TIMEOUT_SECONDS="${CAMERA_READY_TIMEOUT_SECONDS:-75}"
CAMERA_STARTUP_MIN_FPS="${CAMERA_STARTUP_MIN_FPS:-20}"
CAMERA_STARTUP_MAX_FPS="${CAMERA_STARTUP_MAX_FPS:-40}"
CAMERA_STARTUP_STABLE_SAMPLES="${CAMERA_STARTUP_STABLE_SAMPLES:-3}"

HEALTH_CHECK_INTERVAL_SECONDS="${HEALTH_CHECK_INTERVAL_SECONDS:-2}"
CAMERA_LOG_STALE_CHECKS="${CAMERA_LOG_STALE_CHECKS:-5}"
CAMERA_IDLE_ZERO_FPS_LIMIT="${CAMERA_IDLE_ZERO_FPS_LIMIT:-3}"
LOCK_FILE="${LOCK_FILE:-/tmp/nanopi_capture_${UID}_${SESSION_NAME}.lock}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
CAMERA_LOG="${LOG_DIR}/camera_${RUN_STAMP}.log"
UART_LOG="${LOG_DIR}/uart_${RUN_STAMP}.log"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

for command_name in flock tmux stdbuf awk find grep readlink rm sed stat tail wc pgrep; do
    command -v "${command_name}" >/dev/null 2>&1 ||
        fail "required command is missing: ${command_name}"
done
[[ -x "${CAMERA_BIN}" ]] || fail "camera executable not found: ${CAMERA_BIN}"
[[ -x "${UART_BIN}" ]] || fail "UART executable not found: ${UART_BIN}"
[[ -f "${CONFIG_PATH}" ]] || fail "camera config not found: ${CONFIG_PATH}"
[[ -e "${SERIAL_DEV}" ]] || fail "serial device not found: ${SERIAL_DEV}"

case "${TIME_SET_MODE}" in
    idle|every|once|none) ;;
    *) fail "TIME_SET_MODE must be idle, every, once, or none" ;;
esac

exec 9>"${LOCK_FILE}"
flock -n 9 || fail "another autostart supervisor is already running"

session_started=0

window_exists() {
    tmux list-windows -t "${SESSION_NAME}" -F '#W' 2>/dev/null |
        grep -Fxq "$1"
}

wait_for_pane_exit() {
    local target=$1
    local timeout_seconds=$2
    local deadline=$((SECONDS + timeout_seconds))
    while (( SECONDS < deadline )); do
        local dead
        dead="$(tmux display-message -p -t "${target}" '#{pane_dead}' 2>/dev/null || echo 1)"
        [[ "${dead}" == "1" ]] && return 0
        sleep 0.2
    done
    return 1
}

stop_tmux_gracefully() {
    tmux has-session -t "${SESSION_NAME}" 2>/dev/null || return 0

    if window_exists receiver; then
        tmux send-keys -t "${SESSION_NAME}:receiver" C-c 2>/dev/null || true
        wait_for_pane_exit "${SESSION_NAME}:receiver" 5 ||
            echo "[WARN] UART receiver did not exit within 5s" >&2
    fi
    if window_exists camera; then
        tmux send-keys -t "${SESSION_NAME}:camera" C-c 2>/dev/null || true
        wait_for_pane_exit "${SESSION_NAME}:camera" 25 ||
            echo "[WARN] Camera did not exit within 25s" >&2
    fi
    tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if (( session_started == 1 )); then
        stop_tmux_gracefully || true
    fi
    exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Do not destroy a live manual session. Only remove a tmux session whose panes
# have all exited and were kept solely by remain-on-exit.
if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    if tmux list-panes -s -t "${SESSION_NAME}" -F '#{pane_dead}' 2>/dev/null |
       grep -Fxq '0'; then
        fail "live tmux session already exists; stop manual acquisition before enabling autostart v2"
    fi
    echo "[WARN] Removing dead stale tmux session: ${SESSION_NAME}"
    tmux kill-session -t "${SESSION_NAME}"
fi

# A camera process outside the expected tmux session must not be competed with.
if pgrep -f '(^|/)mode2_capture([[:space:]]|$)' >/dev/null 2>&1; then
    fail "mode2_capture is already running outside the managed tmux session"
fi

camera_usb_snapshot() {
    local product_file usb_dir usb_real_dir speed video_nodes
    for product_file in /sys/bus/usb/devices/*/product; do
        [[ -r "${product_file}" ]] || continue
        grep -Fqi -- "${CAMERA_USB_PRODUCT_PATTERN}" "${product_file}" || continue

        usb_dir="${product_file%/product}"
        [[ -r "${usb_dir}/speed" ]] || continue
        speed=""
        IFS= read -r speed <"${usb_dir}/speed" || continue
        # Resolve only the outer /sys/bus/usb/devices symlink. Do not use
        # find -L: sysfs contains back-links that can make recursive walks loop.
        usb_real_dir="$(readlink -f "${usb_dir}" 2>/dev/null || true)"
        [[ -n "${usb_real_dir}" ]] || continue
        video_nodes="$(find "${usb_real_dir}" -maxdepth 4 -type d \
            -regex '.*/video[0-9][0-9]*' 2>/dev/null |
            wc -l || true)"

        if awk -v actual="${speed}" -v minimum="${CAMERA_MIN_USB_SPEED_MBPS}" \
            'BEGIN { exit !(actual + 0 >= minimum + 0) }' &&
           (( video_nodes >= CAMERA_MIN_VIDEO_NODES )); then
            printf '%s speed=%sMbps video_nodes=%s' \
                "$(basename "${usb_dir}")" "${speed}" "${video_nodes}"
            return 0
        fi
    done
    return 1
}

echo "[INFO] Waiting up to ${CAMERA_DEVICE_WAIT_TIMEOUT_SECONDS}s for a stable USB3 RealSense (${CAMERA_MIN_VIDEO_NODES}+ video nodes)..."
device_deadline=$((SECONDS + CAMERA_DEVICE_WAIT_TIMEOUT_SECONDS))
stable_usb_samples=0
last_usb_snapshot=""
while (( SECONDS < device_deadline )); do
    current_usb_snapshot=""
    if current_usb_snapshot="$(camera_usb_snapshot)"; then
        if [[ "${current_usb_snapshot}" == "${last_usb_snapshot}" ]]; then
            stable_usb_samples=$((stable_usb_samples + 1))
        else
            stable_usb_samples=1
            last_usb_snapshot="${current_usb_snapshot}"
            echo "[INFO] USB candidate: ${current_usb_snapshot}"
        fi
        if (( stable_usb_samples >= CAMERA_USB_STABLE_SECONDS )); then
            break
        fi
    else
        stable_usb_samples=0
        last_usb_snapshot=""
    fi
    sleep 1
done
(( stable_usb_samples >= CAMERA_USB_STABLE_SECONDS )) ||
    fail "RealSense did not remain USB3/video-node stable for ${CAMERA_USB_STABLE_SECONDS}s"
echo "[OK] USB camera stable: ${last_usb_snapshot}"

mkdir -p "${LOG_DIR}"

# No managed process is running at this point. Remove the stale IPC object so
# the new UART receiver must establish a fresh camera-control channel.
if [[ -e "${SHM_PATH}" ]]; then
    rm -f -- "${SHM_PATH}" || fail "could not remove stale camera shared memory: ${SHM_PATH}"
fi

printf -v root_q '%q' "${ROOT_DIR}"
printf -v camera_bin_q '%q' "${CAMERA_BIN}"
printf -v config_q '%q' "${CONFIG_PATH}"
printf -v camera_log_q '%q' "${CAMERA_LOG}"
camera_command="set -o pipefail; cd ${root_q}; exec stdbuf -oL -eL ${camera_bin_q} ${config_q} 2>&1 | tee -a ${camera_log_q}"
printf -v camera_command_q '%q' "${camera_command}"

tmux new-session -d -s "${SESSION_NAME}" -n camera \
    "bash -lc ${camera_command_q}"
session_started=1
tmux set-option -w -t "${SESSION_NAME}:camera" remain-on-exit on

echo "[INFO] Camera launched; validating stable framesets..."
camera_ready=0
stable_fps_samples=0
last_stats_count=0
ready_deadline=$((SECONDS + CAMERA_READY_TIMEOUT_SECONDS))
while (( SECONDS < ready_deadline )); do
    pane_dead="$(tmux display-message -p -t "${SESSION_NAME}:camera" '#{pane_dead}' 2>/dev/null || echo 1)"
    if [[ "${pane_dead}" == "1" ]]; then
        tail -n 80 "${CAMERA_LOG}" 2>/dev/null || true
        fail "camera process exited before becoming ready"
    fi

    stats_count="$(grep -Fc '[STATS]' "${CAMERA_LOG}" 2>/dev/null || true)"
    if (( stats_count > last_stats_count )); then
        latest_stats="$(grep -F '[STATS]' "${CAMERA_LOG}" | tail -n 1)"
        latest_fps="$(sed -n 's/.*capture_fps=\([0-9][0-9.]*\).*/\1/p' <<<"${latest_stats}")"
        if [[ -n "${latest_fps}" ]] &&
           awk -v fps="${latest_fps}" -v minimum="${CAMERA_STARTUP_MIN_FPS}" \
               -v maximum="${CAMERA_STARTUP_MAX_FPS}" \
               'BEGIN { exit !(fps + 0 >= minimum + 0 && fps + 0 <= maximum + 0) }'; then
            stable_fps_samples=$((stable_fps_samples + 1))
        else
            stable_fps_samples=0
        fi
        last_stats_count="${stats_count}"
        echo "[INFO] Startup frame validation: fps=${latest_fps:-unknown} stable_samples=${stable_fps_samples}/${CAMERA_STARTUP_STABLE_SAMPLES}"
    fi

    if (( stable_fps_samples >= CAMERA_STARTUP_STABLE_SAMPLES )) &&
       grep -Fq '[CAPTURE] ready' "${CAMERA_LOG}" 2>/dev/null; then
        camera_ready=1
        break
    fi
    sleep 0.1
done

if (( camera_ready != 1 )); then
    tail -n 80 "${CAMERA_LOG}" 2>/dev/null || true
    fail "camera did not produce stable framesets within ${CAMERA_READY_TIMEOUT_SECONDS}s"
fi

printf -v uart_bin_q '%q' "${UART_BIN}"
printf -v serial_dev_q '%q' "${SERIAL_DEV}"
printf -v serial_baud_q '%q' "${SERIAL_BAUD}"
printf -v time_set_mode_q '%q' "${TIME_SET_MODE}"
printf -v uart_log_q '%q' "${UART_LOG}"
uart_command="set -o pipefail; exec ${uart_bin_q} ${serial_dev_q} ${serial_baud_q} ${time_set_mode_q} 2>&1 | tee -a ${uart_log_q}"
printf -v uart_command_q '%q' "${uart_command}"

tmux new-window -d -t "${SESSION_NAME}" -n receiver \
    "bash -lc ${uart_command_q}"
tmux set-option -w -t "${SESSION_NAME}:receiver" remain-on-exit on
tmux select-window -t "${SESSION_NAME}:receiver"

receiver_ready=0
receiver_deadline=$((SECONDS + 8))
while (( SECONDS < receiver_deadline )); do
    receiver_dead="$(tmux display-message -p -t "${SESSION_NAME}:receiver" '#{pane_dead}' 2>/dev/null || echo 1)"
    if [[ "${receiver_dead}" == "1" ]]; then
        tail -n 80 "${UART_LOG}" 2>/dev/null || true
        fail "UART receiver exited immediately"
    fi
    if [[ -e "${SHM_PATH}" ]] &&
       grep -Fq '[OK] Camera shared memory connected' "${UART_LOG}" 2>/dev/null; then
        receiver_ready=1
        break
    fi
    sleep 0.1
done
if (( receiver_ready != 1 )); then
    tail -n 80 "${UART_LOG}" 2>/dev/null || true
    fail "UART receiver did not establish camera shared memory within 8s"
fi

echo "[OK] Autostart v2 ready; camera frames are stable and UART receiver is running"
echo "[INFO] Camera log: ${CAMERA_LOG}"
echo "[INFO] UART log:   ${UART_LOG}"
echo "[INFO] Attach with: tmux attach-session -t ${SESSION_NAME}"

last_camera_log_size="$(stat -c '%s' "${CAMERA_LOG}" 2>/dev/null || echo 0)"
stale_log_checks=0
idle_zero_fps_samples=0

while true; do
    for window in camera receiver; do
        pane_dead="$(tmux display-message -p -t "${SESSION_NAME}:${window}" '#{pane_dead}' 2>/dev/null || echo 1)"
        if [[ "${pane_dead}" == "1" ]]; then
            if [[ "${window}" == "camera" ]]; then
                tail -n 80 "${CAMERA_LOG}" 2>/dev/null || true
            else
                tail -n 80 "${UART_LOG}" 2>/dev/null || true
            fi
            fail "tmux window '${window}' exited"
        fi
    done

    current_camera_log_size="$(stat -c '%s' "${CAMERA_LOG}" 2>/dev/null || echo 0)"
    if [[ "${current_camera_log_size}" == "${last_camera_log_size}" ]]; then
        stale_log_checks=$((stale_log_checks + 1))
    else
        stale_log_checks=0
        last_camera_log_size="${current_camera_log_size}"
    fi
    if (( stale_log_checks >= CAMERA_LOG_STALE_CHECKS )); then
        fail "camera process is alive but its log/stats stopped advancing"
    fi

    latest_stats="$(tail -n 30 "${CAMERA_LOG}" 2>/dev/null | grep -F '[STATS]' | tail -n 1 || true)"
    latest_rec="$(sed -n 's/.*rec=\([^ ]*\).*/\1/p' <<<"${latest_stats}")"
    latest_fps="$(sed -n 's/.*capture_fps=\([0-9][0-9.]*\).*/\1/p' <<<"${latest_stats}")"
    if [[ "${latest_rec}" == "OFF" && -n "${latest_fps}" ]] &&
       awk -v fps="${latest_fps}" 'BEGIN { exit !(fps + 0 < 5.0) }'; then
        idle_zero_fps_samples=$((idle_zero_fps_samples + 1))
    else
        idle_zero_fps_samples=0
    fi
    if (( idle_zero_fps_samples >= CAMERA_IDLE_ZERO_FPS_LIMIT )); then
        fail "camera remained at near-zero FPS while idle; requesting a clean service restart"
    fi
    if [[ "${latest_rec}" == "ON" && -n "${latest_fps}" ]] &&
       awk -v fps="${latest_fps}" 'BEGIN { exit !(fps + 0 < 5.0) }'; then
        echo "[ERROR] Recording is active but capture_fps=${latest_fps}; deferring restart until recording stops" >&2
    fi

    sleep "${HEALTH_CHECK_INTERVAL_SECONDS}"
done
