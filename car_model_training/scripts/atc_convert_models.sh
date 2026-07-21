#!/usr/bin/env bash
# Convert exported LaneNet/YOLO ONNX files to Ascend310B4 .om deployment models.
set -euo pipefail

SOC_VERSION="${SOC_VERSION:-Ascend310B4}"
LANENET_ONNX="${LANENET_ONNX:-lanenet/runs/lanenet_output/lane_custom_final_batch_8.onnx}"
YOLO_ONNX="${YOLO_ONNX:-yolov5/runs/train/car_yolo/weights/best.onnx}"
OUTPUT_DIR="${OUTPUT_DIR:-deploy_models}"
TE_PARALLEL_COMPILER="${TE_PARALLEL_COMPILER:-1}"
export TE_PARALLEL_COMPILER

mkdir -p "${OUTPUT_DIR}"

if [[ -n "${CANN_INSTALL_PATH:-}" ]]; then
  CANN_SET_ENV="${CANN_INSTALL_PATH}/ascend-toolkit/set_env.sh"
  if [[ ! -d "${CANN_INSTALL_PATH}/ascend-toolkit" ]]; then
    echo "ERROR: CANN_INSTALL_PATH does not contain ascend-toolkit: ${CANN_INSTALL_PATH}"
    echo "Use the real install root, for example: /usr/local/Ascend"
    exit 1
  fi
  if [[ ! -f "${CANN_SET_ENV}" ]]; then
    echo "ERROR: CANN set_env.sh not found: ${CANN_SET_ENV}"
    exit 1
  fi
  # shellcheck source=/dev/null
  source "${CANN_SET_ENV}"

  CANN_DEVLIB_FOUND=0
  for CANN_DEVLIB in \
    "${CANN_INSTALL_PATH}/ascend-toolkit/latest/x86_64-linux/devlib" \
    "${CANN_INSTALL_PATH}"/ascend-toolkit/*/x86_64-linux/devlib; do
    if [[ -d "${CANN_DEVLIB}" ]]; then
      export LD_LIBRARY_PATH="${CANN_DEVLIB}:${LD_LIBRARY_PATH:-}"
      CANN_DEVLIB_FOUND=1
      break
    fi
  done
  if [[ "${CANN_DEVLIB_FOUND}" -eq 0 ]]; then
    echo "ERROR: CANN x86_64-linux/devlib not found under ${CANN_INSTALL_PATH}/ascend-toolkit"
    exit 1
  fi
fi

if ! command -v atc >/dev/null 2>&1; then
  echo "ERROR: atc not found. Run Ascend CANN set_env.sh first, for example:"
  echo "  export CANN_INSTALL_PATH=/path/to/your/cann/install"
  echo "  source \${CANN_INSTALL_PATH}/ascend-toolkit/set_env.sh"
  echo "  source /usr/local/Ascend/ascend-toolkit/set_env.sh"
  echo "or:"
  echo "  source /usr/local/Ascend/ascend-toolkit/latest/set_env.sh"
  exit 1
fi

PYTHON_BIN=""
if command -v python >/dev/null 2>&1; then
  PYTHON_BIN="python"
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_BIN="python3"
fi

if [[ -z "${PYTHON_BIN}" ]]; then
  echo "ERROR: python not found. Activate the training/ATC Python environment first."
  exit 1
fi

if ! "${PYTHON_BIN}" -c "import sympy" >/dev/null 2>&1; then
  echo "ERROR: Python package sympy is required by ATC/TBE but is not installed in the current environment."
  echo "Current python: $(${PYTHON_BIN} -c 'import sys; print(sys.executable)')"
  echo "Install it with:"
  echo "  ${PYTHON_BIN} -m pip install sympy"
  exit 1
fi

echo "SOC_VERSION=${SOC_VERSION}"
echo "OUTPUT_DIR=${OUTPUT_DIR}"
echo "TE_PARALLEL_COMPILER=${TE_PARALLEL_COMPILER}"

if [[ -f "${LANENET_ONNX}" ]]; then
  echo "Converting LaneNet: ${LANENET_ONNX}"
  atc \
    --model="${LANENET_ONNX}" \
    --framework=5 \
    --output="${OUTPUT_DIR}/lanenet" \
    --input_format=NCHW \
    --input_shape="images:1,3,256,512" \
    --soc_version="${SOC_VERSION}"
else
  echo "Skip LaneNet: ${LANENET_ONNX} not found"
fi

if [[ -f "${YOLO_ONNX}" ]]; then
  echo "Converting YOLOv5: ${YOLO_ONNX}"
  atc \
    --model="${YOLO_ONNX}" \
    --framework=5 \
    --output="${OUTPUT_DIR}/yolo" \
    --input_format=NCHW \
    --input_shape="images:1,3,640,640" \
    --soc_version="${SOC_VERSION}"
else
  echo "Skip YOLOv5: ${YOLO_ONNX} not found"
fi

echo "Finished. Copy generated .om files from ${OUTPUT_DIR}/ to the car project's weights/ directory."
