#!/bin/bash
# 脚本功能：将编译完成的可执行文件上传到远程设备

set -e

WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${WORK_DIR}/../out"

# 默认配置（可被命令行参数覆盖）
REMOTE_IP="${1:-192.168.206.50}"
REMOTE_USER="${2:-root}"
REMOTE_PATH="${3:-/home/root/}"
TARGET_FILE="${4:-}"

error_exit() {
    echo -e "\033[31m[ERROR] $1\033[0m"
    exit 1
}

info_echo() {
    echo -e "\033[32m[INFO] $1\033[0m"
}

# 如果没传目标文件名，默认使用 user 目录上一级目录名（与 CMake project 名一致）
if [ -z "${TARGET_FILE}" ]; then
    TARGET_FILE="$(basename "$(dirname "${WORK_DIR}")")"
fi

LOCAL_FILE="${OUT_DIR}/${TARGET_FILE}"

[ -d "${OUT_DIR}" ] || error_exit "编译输出目录不存在: ${OUT_DIR}"
[ -f "${LOCAL_FILE}" ] || error_exit "未找到编译产物: ${LOCAL_FILE}"

info_echo "准备上传: ${LOCAL_FILE}"
info_echo "目标位置: ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}"

scp -O "${LOCAL_FILE}" "${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}" || error_exit "上传失败，请检查网络或远程权限。"

info_echo "✅ 上传成功"
