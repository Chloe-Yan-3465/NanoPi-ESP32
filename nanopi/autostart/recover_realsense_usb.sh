#!/usr/bin/env bash
set -Eeuo pipefail

# librealsense's RSUSB backend detaches the kernel video interfaces while the
# capture process owns the camera. If that process dies abruptly, the USB
# device can remain online at 5 Gbps without recreating /dev/video*. This
# root-only pre-start hook re-authorizes only that incomplete, idle device.
PRODUCT_PATTERN="${CAMERA_USB_PRODUCT_PATTERN:-RealSense}"
MIN_USB_SPEED_MBPS="${CAMERA_MIN_USB_SPEED_MBPS:-5000}"
MIN_VIDEO_NODES="${CAMERA_MIN_VIDEO_NODES:-6}"
ENUMERATION_GRACE_SECONDS="${CAMERA_ENUMERATION_GRACE_SECONDS:-5}"
RECOVERY_WAIT_SECONDS="${CAMERA_RECOVERY_WAIT_SECONDS:-20}"

[[ "${EUID}" -eq 0 ]] || {
    echo "[ERROR] RealSense recovery hook must run as root" >&2
    exit 1
}

if pgrep -f '(^|/)mode2_capture([[:space:]]|$)' >/dev/null 2>&1; then
    echo "[INFO] RealSense recovery skipped: mode2_capture is already running"
    exit 0
fi

video_node_count() {
    local usb_dir=$1
    local usb_real_dir
    usb_real_dir="$(readlink -f "${usb_dir}" 2>/dev/null || true)"
    [[ -n "${usb_real_dir}" ]] || {
        printf '0'
        return
    }
    find "${usb_real_dir}" -maxdepth 4 -type d \
        -regex '.*/video[0-9][0-9]*' 2>/dev/null | wc -l
}

for product_file in /sys/bus/usb/devices/*/product; do
    [[ -r "${product_file}" ]] || continue
    grep -Fqi -- "${PRODUCT_PATTERN}" "${product_file}" || continue

    usb_dir="${product_file%/product}"
    [[ -r "${usb_dir}/speed" && -w "${usb_dir}/authorized" ]] || continue
    IFS= read -r speed <"${usb_dir}/speed" || continue
    awk -v actual="${speed}" -v minimum="${MIN_USB_SPEED_MBPS}" \
        'BEGIN { exit !(actual + 0 >= minimum + 0) }' || continue

    nodes="$(video_node_count "${usb_dir}")"
    if (( nodes >= MIN_VIDEO_NODES )); then
        echo "[INFO] RealSense USB preflight healthy: $(basename "${usb_dir}") speed=${speed}Mbps video_nodes=${nodes}"
        exit 0
    fi

    # Avoid resetting a normally enumerating camera during early boot.
    for ((second = 0; second < ENUMERATION_GRACE_SECONDS; second++)); do
        sleep 1
        nodes="$(video_node_count "${usb_dir}")"
        if (( nodes >= MIN_VIDEO_NODES )); then
            echo "[INFO] RealSense completed normal enumeration: $(basename "${usb_dir}") video_nodes=${nodes}"
            exit 0
        fi
    done

    echo "[WARN] RealSense is USB3-online but incomplete (video_nodes=${nodes}); re-authorizing $(basename "${usb_dir}")"
    printf '0' >"${usb_dir}/authorized"
    sleep 2
    printf '1' >"${usb_dir}/authorized"

    for ((second = 0; second < RECOVERY_WAIT_SECONDS; second++)); do
        sleep 1
        nodes="$(video_node_count "${usb_dir}")"
        if (( nodes >= MIN_VIDEO_NODES )); then
            echo "[OK] RealSense USB recovery complete: $(basename "${usb_dir}") speed=${speed}Mbps video_nodes=${nodes}"
            exit 0
        fi
    done

    echo "[WARN] RealSense USB recovery did not recreate ${MIN_VIDEO_NODES} video nodes; supervisor will continue its normal wait"
    exit 0
done

echo "[INFO] No USB3 RealSense present during pre-start; supervisor will wait for it"
