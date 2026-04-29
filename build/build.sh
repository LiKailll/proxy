#!/bin/bash

# 获取脚本所在的目录
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo $SCRIPT_DIR
# 切换到项目根目录
cd "${SCRIPT_DIR}/.."
# 设置项目目录变量
PROJECT_DIR="$(pwd)"
echo $PROJECT_DIR

# 定义四个版本的 Makefile 文件名
proxy_V1="${PROJECT_DIR}/proxy_3slot"
proxy_V2="${PROJECT_DIR}/proxy_5slot"
proxy_V3="${PROJECT_DIR}/proxy_8slot"
proxy_V4="${PROJECT_DIR}/proxy_fudanwei/proxy_8slot"

# 编译每个版本的代理项目
for i in {1..4}; do
    case $i in
        1) proxy_ver="$proxy_V1";;
        2) proxy_ver="$proxy_V2";;
        3) proxy_ver="$proxy_V3";;
        4) proxy_ver="$proxy_V4";;
    esac

    # 调用 make 命令编译项目
    echo $proxy_ver
    cd "${proxy_ver}"
    make
    cd "${PROJECT_DIR}"
    
    # 检查 make 命令是否成功执行
    if [ $? -ne 0 ]; then
        echo "*** Error during make process for version $i [$?]"
        EXIT_CODE=$?
        exit $EXIT_CODE
    fi
done

# 输出成功信息
if [ "$EXIT_CODE" -eq 0 ]; then
    echo "All build processes completed successfully."
else
    echo "Some build processes failed with exit code $EXIT_CODE."
fi

# 设置退出代码并退出脚本
exit $EXIT_CODE