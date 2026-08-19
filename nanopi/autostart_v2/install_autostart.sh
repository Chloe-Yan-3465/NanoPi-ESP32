#!/usr/bin/env bash
set -Eeuo pipefail

# v2 replaces the implementation of the existing service name. Deliberately
# do not create a second systemd unit that could run beside v1.
SERVICE_NAME="nanopi-capture.service"
SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="${1:-/home/pi}"
RUN_USER="${2:-pi}"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

[[ "${EUID}" -eq 0 ]] || fail "run this installer with sudo"
[[ "${APP_DIR}" == /* ]] || fail "APP_DIR must be an absolute path"
[[ "${RUN_USER}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || fail "invalid service user: ${RUN_USER}"
id "${RUN_USER}" >/dev/null 2>&1 || fail "user does not exist: ${RUN_USER}"
RUN_GROUP="$(id -gn "${RUN_USER}")"

AUTOSTART_TEMPLATE="${SOURCE_DIR}/start_tmux_autostart.sh"
RECOVERY_TEMPLATE="${SOURCE_DIR}/recover_realsense_usb.sh"
SERVICE_TEMPLATE="${SOURCE_DIR}/${SERVICE_NAME}"
AUTOSTART_TARGET="${APP_DIR}/start_tmux_autostart_v2.sh"
RECOVERY_TARGET="/usr/local/sbin/nanopi-recover-realsense-v2"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"

[[ -f "${AUTOSTART_TEMPLATE}" ]] || fail "v2 autostart script not found"
[[ -f "${RECOVERY_TEMPLATE}" ]] || fail "v2 USB recovery script not found"
[[ -f "${SERVICE_TEMPLATE}" ]] || fail "v2 service template not found"
[[ -x "${APP_DIR}/build/mode2_capture" ]] || fail "camera executable is missing"
[[ -x "${APP_DIR}/build/uart_camera_receiver" ]] || fail "UART executable is missing"
[[ -f "${APP_DIR}/camera_cap/config.yaml" ]] || fail "camera config is missing"

WORK_DIR="$(mktemp -d)"
trap 'rm -rf -- "${WORK_DIR}"' EXIT
cp -- "${SERVICE_TEMPLATE}" "${WORK_DIR}/${SERVICE_NAME}"

BACKUP_DIR="${APP_DIR}/autostart-backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p -- "${BACKUP_DIR}"
for existing_file in "${AUTOSTART_TARGET}" "${RECOVERY_TARGET}" "${SERVICE_TARGET}"; do
    if [[ -e "${existing_file}" ]]; then
        cp -a -- "${existing_file}" "${BACKUP_DIR}/"
    fi
done

install -o "${RUN_USER}" -g "${RUN_GROUP}" -m 0755 \
    "${AUTOSTART_TEMPLATE}" "${AUTOSTART_TARGET}"
install -o root -g root -m 0755 \
    "${RECOVERY_TEMPLATE}" "${RECOVERY_TARGET}"

escaped_app_dir="${APP_DIR//&/\\&}"
escaped_autostart_target="${AUTOSTART_TARGET//&/\\&}"
sed \
    -e "s|^User=.*|User=${RUN_USER}|" \
    -e "s|^Group=.*|Group=${RUN_GROUP}|" \
    -e "s|^WorkingDirectory=.*|WorkingDirectory=${escaped_app_dir}|" \
    -e "s|^Environment=HOME=.*|Environment=HOME=${escaped_app_dir}|" \
    -e "s|^ExecStart=.*|ExecStart=${escaped_autostart_target}|" \
    "${WORK_DIR}/${SERVICE_NAME}" >"${WORK_DIR}/${SERVICE_NAME}.rendered"
install -o root -g root -m 0644 \
    "${WORK_DIR}/${SERVICE_NAME}.rendered" "${SERVICE_TARGET}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"
systemctl restart "${SERVICE_NAME}"

echo "[OK] Installed and started autostart v2 through ${SERVICE_NAME}"
echo "[INFO] Status:  systemctl status ${SERVICE_NAME}"
echo "[INFO] Logs:    journalctl -u ${SERVICE_NAME} -f"
echo "[INFO] Session: sudo -u ${RUN_USER} tmux attach-session -t mode2_capture_20260808"
