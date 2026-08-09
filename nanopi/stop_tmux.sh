#!/usr/bin/env bash
set -Eeuo pipefail

SESSION_NAME="${SESSION_NAME:-mode2_capture_20260808}"

if ! command -v tmux >/dev/null 2>&1; then
    echo "[ERROR] tmux is not installed" >&2
    exit 1
fi

if ! tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo "[INFO] tmux session does not exist: ${SESSION_NAME}"
    exit 0
fi

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

echo "[INFO] Stopping UART receiver first..."
if window_exists receiver; then
    tmux send-keys -t "${SESSION_NAME}:receiver" C-c
    if ! wait_for_pane_exit "${SESSION_NAME}:receiver" 5; then
        echo "[WARN] UART receiver did not exit within 5s"
    fi
fi

echo "[INFO] Stopping camera process..."
if window_exists camera; then
    tmux send-keys -t "${SESSION_NAME}:camera" C-c
    if ! wait_for_pane_exit "${SESSION_NAME}:camera" 20; then
        echo "[WARN] Camera did not exit within 20s; closing tmux session"
    fi
fi

tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
echo "[OK] tmux session stopped: ${SESSION_NAME}"
