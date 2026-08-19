#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TARGET_HOST="${1:-}"
TARGET_USER="${2:-pi}"
APP_DIR="${3:-/home/pi}"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

[[ -n "${TARGET_HOST}" ]] || {
    echo "Usage: $0 <NanoPi-IP-or-hostname> [ssh-user] [app-dir]" >&2
    echo "Example: $0 192.168.8.68 pi /home/pi" >&2
    exit 2
}
[[ "${TARGET_HOST}" =~ ^[A-Za-z0-9._:-]+$ ]] || fail "invalid target host"
[[ "${TARGET_USER}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || fail "invalid SSH user"
[[ "${APP_DIR}" == /* && "${APP_DIR}" != "/" ]] || fail "app-dir must be an absolute non-root path"

for command_name in ssh scp; do
    command -v "${command_name}" >/dev/null 2>&1 ||
        fail "required command is missing: ${command_name}"
done
for file_name in start_tmux_autostart.sh recover_realsense_usb.sh \
    nanopi-capture.service install_autostart.sh README.md; do
    [[ -f "${SCRIPT_DIR}/${file_name}" ]] || fail "missing bundle file: ${file_name}"
done

TARGET="${TARGET_USER}@${TARGET_HOST}"
REMOTE_BUNDLE_DIR="${APP_DIR}/autostart_v2"
printf -v remote_bundle_q '%q' "${REMOTE_BUNDLE_DIR}"
printf -v app_dir_q '%q' "${APP_DIR}"
printf -v target_user_q '%q' "${TARGET_USER}"

echo "[INFO] Uploading autostart v2 to ${TARGET}:${REMOTE_BUNDLE_DIR}"
ssh "${TARGET}" "mkdir -p -- ${remote_bundle_q}"
scp \
    "${SCRIPT_DIR}/start_tmux_autostart.sh" \
    "${SCRIPT_DIR}/recover_realsense_usb.sh" \
    "${SCRIPT_DIR}/nanopi-capture.service" \
    "${SCRIPT_DIR}/install_autostart.sh" \
    "${SCRIPT_DIR}/README.md" \
    "${TARGET}:${REMOTE_BUNDLE_DIR}/"

echo "[INFO] Validating and installing on ${TARGET_HOST}; sudo may ask for the NanoPi password"
ssh -t "${TARGET}" \
    "bash -n ${remote_bundle_q}/start_tmux_autostart.sh ${remote_bundle_q}/recover_realsense_usb.sh ${remote_bundle_q}/install_autostart.sh && sudo bash ${remote_bundle_q}/install_autostart.sh ${app_dir_q} ${target_user_q}"

echo "[OK] Autostart v2 deployed to ${TARGET_HOST}"
echo "[INFO] Verify with: ssh ${TARGET} 'systemctl status nanopi-capture.service'"
