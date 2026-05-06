#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-host"
DEFAULT_LIB_DIR="${SCRIPT_DIR}/venv/lib/python3.11/site-packages/onnxruntime/capi"

if ! command -v cmake >/dev/null 2>&1; then
  echo "cmake is missing. Install host build tools first:"
  echo "  sudo apt update"
  echo "  sudo apt install -y build-essential cmake"
  exit 1
fi

if ! command -v g++ >/dev/null 2>&1; then
  echo "g++ is missing. Install host build tools first:"
  echo "  sudo apt update"
  echo "  sudo apt install -y build-essential cmake"
  exit 1
fi

ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-}"
ONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR:-}"
ONNXRUNTIME_LIB_DIR="${ONNXRUNTIME_LIB_DIR:-$DEFAULT_LIB_DIR}"

if [[ -z "${ONNXRUNTIME_ROOT}" && -z "${ONNXRUNTIME_INCLUDE_DIR}" ]]; then
  echo "ONNX Runtime headers are not configured."
  echo "This repo already contains a runtime .so from the Python wheel at:"
  echo "  ${DEFAULT_LIB_DIR}"
  echo
  echo "But the wheel does not include C/C++ headers, so you still need the SDK headers."
  echo "Install an aarch64 ONNX Runtime package on the host, then rerun with one of:"
  echo "  ONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-<version> ./onnx/build_host.sh"
  echo "  ONNXRUNTIME_INCLUDE_DIR=/path/to/onnxruntime/include ONNXRUNTIME_LIB_DIR=${DEFAULT_LIB_DIR} ./onnx/build_host.sh"
  exit 1
fi

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
  -DONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}" \
  -DONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR}" \
  -DONNXRUNTIME_LIB_DIR="${ONNXRUNTIME_LIB_DIR}"

cmake --build "${BUILD_DIR}" -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"

echo
echo "Built:"
echo "  ${BUILD_DIR}/run_random_inference"
echo
echo "Example:"
echo "  ${BUILD_DIR}/run_random_inference ${SCRIPT_DIR}/marl_agent.onnx"
