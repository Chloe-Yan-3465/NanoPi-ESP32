#!/usr/bin/env bash
set -Eeuo pipefail

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

ORIGINAL_SCRIPT="${APP_DIR}/start_tmux.sh"
AUTOSTART_TEMPLATE="${SOURCE_DIR}/start_tmux_autostart.sh"
SERVICE_TEMPLATE="${SOURCE_DIR}/${SERVICE_NAME}"
AUTOSTART_TARGET="${APP_DIR}/start_tmux_autostart.sh"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"

[[ -f "${ORIGINAL_SCRIPT}" ]] || fail "original script not found: ${ORIGINAL_SCRIPT}"
[[ -f "${AUTOSTART_TEMPLATE}" ]] || fail "autostart template not found: ${AUTOSTART_TEMPLATE}"
[[ -f "${SERVICE_TEMPLATE}" ]] || fail "service template not found: ${SERVICE_TEMPLATE}"
[[ -x "${APP_DIR}/build/mode2_capture" ]] || fail "camera executable is missing"
[[ -x "${APP_DIR}/build/uart_camera_receiver" ]] || fail "UART executable is missing"
[[ -f "${APP_DIR}/camera_cap/config.yaml" ]] || fail "camera config is missing"

WORK_DIR="$(mktemp -d)"
trap 'rm -rf -- "${WORK_DIR}"' EXIT
cp -- "${AUTOSTART_TEMPLATE}" "${WORK_DIR}/start_tmux_autostart.sh"
cp -- "${SERVICE_TEMPLATE}" "${WORK_DIR}/${SERVICE_NAME}"

BACKUP_DIR="${APP_DIR}/autostart-backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p -- "${BACKUP_DIR}"
if [[ -e "${AUTOSTART_TARGET}" ]]; then
    cp -a -- "${AUTOSTART_TARGET}" "${BACKUP_DIR}/"
fi
if [[ -e "${SERVICE_TARGET}" ]]; then
    cp -a -- "${SERVICE_TARGET}" "${BACKUP_DIR}/"
fi

# Make the requested copy first, then install the modified autostart version.
cp -p -- "${ORIGINAL_SCRIPT}" "${AUTOSTART_TARGET}"
install -o "${RUN_USER}" -g "${RUN_GROUP}" -m 0755 \
    "${WORK_DIR}/start_tmux_autostart.sh" "${AUTOSTART_TARGET}"

escaped_app_dir="${APP_DIR//&/\\&}"
sed \
    -e "s|^User=.*|User=${RUN_USER}|" \
    -e "s|^Group=.*|Group=${RUN_GROUP}|" \
    -e "s|^WorkingDirectory=.*|WorkingDirectory=${escaped_app_dir}|" \
    -e "s|^Environment=HOME=.*|Environment=HOME=${escaped_app_dir}|" \
    -e "s|^ExecStart=.*|ExecStart=${escaped_app_dir}/start_tmux_autostart.sh|" \
    "${WORK_DIR}/${SERVICE_NAME}" >"${WORK_DIR}/${SERVICE_NAME}.rendered"
install -o root -g root -m 0644 \
    "${WORK_DIR}/${SERVICE_NAME}.rendered" "${SERVICE_TARGET}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"
systemctl restart "${SERVICE_NAME}"

echo "[OK] Installed and started ${SERVICE_NAME}"
echo "[INFO] Status:  systemctl status ${SERVICE_NAME}"
echo "[INFO] Logs:    journalctl -u ${SERVICE_NAME} -f"
echo "[INFO] Session: sudo -u ${RUN_USER} tmux attach-session -t mode2_capture_20260808"
