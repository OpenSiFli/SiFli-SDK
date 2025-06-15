#!/bin/bash
# 通用构建脚本 - 处理所有动态生成的构建Job

set -e

# 读取环境变量
PROJECT_PATH=${PROJECT_PATH}
BOARD=${BOARD}  # 原始board名称（用于目录名）
SCONS_BOARD=${SCONS_BOARD}  # scons命令使用的board名称
BUILD_ARGS=${BUILD_ARGS}
LOG_NAME=${LOG_NAME}
PROJECT_TYPE=${PROJECT_TYPE}
PARALLEL_JOBS=${PARALLEL_JOBS:-8}

echo "============================================"
echo "构建项目: ${PROJECT_PATH}"
echo "开发板: ${BOARD:-N/A}"
echo "Scons Board: ${SCONS_BOARD:-N/A}"
echo "类型: ${PROJECT_TYPE}"
echo "构建参数: ${BUILD_ARGS}"
echo "============================================"

# 创建必要目录
mkdir -p ci_build_logs artifacts

# 获取绝对路径
ROOT_DIR=$(pwd)
LOG_FILE="${ROOT_DIR}/ci_build_logs/${LOG_NAME}.log"

# 切换到项目目录
cd "${PROJECT_PATH}"

# 构建命令
BUILD_CMD="scons ${BUILD_ARGS} -j${PARALLEL_JOBS}"

echo "[$(date)] 开始构建: ${BUILD_CMD}" | tee "${LOG_FILE}"

# 执行构建
if ${BUILD_CMD} >> "${LOG_FILE}" 2>&1; then
    echo "[$(date)] ✅ 构建成功" | tee -a "${LOG_FILE}"
else
    echo "[$(date)] ❌ 构建失败" | tee -a "${LOG_FILE}"
    exit 1
fi

# 返回根目录
cd "${ROOT_DIR}"

# 收集构建产物
if [ "${PROJECT_TYPE}" = "common" ] && [ -n "${BOARD}" ]; then
    # 通用项目构建目录
    # 确保构建目录名有_hcpu后缀
    if [[ "${BOARD}" == *"hcpu"* ]]; then
        BUILD_DIR="${ROOT_DIR}/${PROJECT_PATH}/build_${BOARD}"
    else
        BUILD_DIR="${ROOT_DIR}/${PROJECT_PATH}/build_${BOARD}_hcpu"
    fi
else
    # 普通项目构建目录
    BUILD_DIR="${ROOT_DIR}/${PROJECT_PATH}/build"
fi

if [ -d "${BUILD_DIR}" ]; then
    mkdir -p "${ROOT_DIR}/artifacts/${LOG_NAME}"
    find "${BUILD_DIR}" -name "*.bin" -o -name "*.hex" -o -name "*.elf" -o -name "*.axf" -o -name "*.map" | while read file; do
        cp "$file" "${ROOT_DIR}/artifacts/${LOG_NAME}/" 2>/dev/null || true
    done
    echo "✅ 构建产物已收集到 artifacts/${LOG_NAME}/"
else
    echo "⚠️  构建产物目录不存在: ${BUILD_DIR}"
fi

echo "构建完成！"
