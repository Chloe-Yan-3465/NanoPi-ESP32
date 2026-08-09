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
CAMERA_READY_TIMEOUT_SECONDS="${CAMERA_READY_TIMEOUT_SECONDS:-30}"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/logs}"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
CAMERA_LOG="${LOG_DIR}/camera_${RUN_STAMP}.log"
UART_LOG="${LOG_DIR}/uart_${RUN_STAMP}.log"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

command -v tmux >/dev/null 2>&1 || fail "tmux is not installed"
command -v stdbuf >/dev/null 2>&1 || fail "stdbuf is not available"
[[ -x "${CAMERA_BIN}" ]] || fail "camera executable not found: ${CAMERA_BIN}"
[[ -x "${UART_BIN}" ]] || fail "UART executable not found: ${UART_BIN}"
[[ -f "${CONFIG_PATH}" ]] || fail "camera config not found: ${CONFIG_PATH}"
[[ -e "${SERIAL_DEV}" ]] || fail "serial device not found: ${SERIAL_DEV}"

case "${TIME_SET_MODE}" in
    idle|every|once|none) ;;
    *) fail "TIME_SET_MODE must be idle, every, once, or none" ;;
esac

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo "[INFO] tmux session already exists: ${SESSION_NAME}"
    exec tmux attach-session -t "${SESSION_NAME}"
fi

mkdir -p "${LOG_DIR}"

old_shm_inode=""
if [[ -e "${SHM_PATH}" ]]; then
    old_shm_inode="$(stat -c '%i' "${SHM_PATH}" 2>/dev/null || true)"
fi

printf -v root_q '%q' "${ROOT_DIR}"
printf -v camera_bin_q '%q' "${CAMERA_BIN}"
printf -v config_q '%q' "${CONFIG_PATH}"
printf -v camera_log_q '%q' "${CAMERA_LOG}"
camera_command="set -o pipefail; cd ${root_q}; exec stdbuf -oL -eL ${camera_bin_q} ${config_q} 2>&1 | tee -a ${camera_log_q}"
printf -v camera_command_q '%q' "${camera_command}"

# Start the camera as the first and only process in the new tmux session.
tmux new-session -d -s "${SESSION_NAME}" -n camera \
    "bash -lc ${camera_command_q}"
tmux set-option -w -t "${SESSION_NAME}:camera" remain-on-exit on

cleanup_failed_start() {
    local status=$?
    if (( status != 0 )); then
        tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
    fi
    exit "${status}"
}
trap cleanup_failed_start ERR INT TERM

echo "[INFO] Camera started first; waiting for fresh shared memory and camera readiness..."
camera_ready=0
deadline=$((SECONDS + CAMERA_READY_TIMEOUT_SECONDS))
while (( SECONDS < deadline )); do
    pane_dead="$(tmux display-message -p -t "${SESSION_NAME}:camera" '#{pane_dead}' 2>/dev/null || echo 1)"
    if [[ "${pane_dead}" == "1" ]]; then
        echo "[ERROR] Camera process exited before becoming ready" >&2
        tail -n 80 "${CAMERA_LOG}" 2>/dev/null || true
        false
    fi

    current_shm_inode=""
    if [[ -e "${SHM_PATH}" ]]; then
        current_shm_inode="$(stat -c '%i' "${SHM_PATH}" 2>/dev/null || true)"
    fi
    if [[ -n "${current_shm_inode}" &&
          ( -z "${old_shm_inode}" || "${current_shm_inode}" != "${old_shm_inode}" ) ]] &&
       grep -Fq '[CAPTURE] ready' "${CAMERA_LOG}" 2>/dev/null; then
        camera_ready=1
        break
    fi
    sleep 0.1
done

if (( camera_ready != 1 )); then
    echo "[ERROR] Camera was not ready within ${CAMERA_READY_TIMEOUT_SECONDS}s" >&2
    tail -n 80 "${CAMERA_LOG}" 2>/dev/null || true
    false
fi

printf -v uart_bin_q '%q' "${UART_BIN}"
printf -v serial_dev_q '%q' "${SERIAL_DEV}"
printf -v serial_baud_q '%q' "${SERIAL_BAUD}"
printf -v time_set_mode_q '%q' "${TIME_SET_MODE}"
printf -v uart_log_q '%q' "${UART_LOG}"
uart_command="set -o pipefail; exec sudo ${uart_bin_q} ${serial_dev_q} ${serial_baud_q} ${time_set_mode_q} 2>&1 | tee -a ${uart_log_q}"
printf -v uart_command_q '%q' "${uart_command}"

# UART starts only after the camera has created fresh shared memory and is ready.
tmux new-window -d -t "${SESSION_NAME}" -n receiver \
    "bash -lc ${uart_command_q}"
tmux set-option -w -t "${SESSION_NAME}:receiver" remain-on-exit on
tmux select-window -t "${SESSION_NAME}:receiver"

trap - ERR INT TERM
echo "[OK] Camera ready; UART receiver started with set_mode=${TIME_SET_MODE}"
echo "[INFO] Camera log: ${CAMERA_LOG}"
echo "[INFO] UART log:   ${UART_LOG}"
echo "[INFO] If sudo asks for a password, enter it in the receiver window."
exec tmux attach-session -t "${SESSION_NAME}"
