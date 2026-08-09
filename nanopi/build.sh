#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
# Neo3 Plus has 1 GB RAM. Two compile jobs are a conservative default.
BUILD_JOBS="${BUILD_JOBS:-2}"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS}"

echo
echo "Build completed:"
echo "  ${BUILD_DIR}/mode2_capture"
echo "  ${BUILD_DIR}/uart_camera_receiver"
