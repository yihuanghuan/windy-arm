#!/bin/bash

# Docker 编译验证脚本
# 用法: ./build.sh [verify|build]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

MODE=${1:-verify}

echo "========================================="
echo "Docker 编译验证脚本"
echo "模式: $MODE"
echo "项目根目录: $PROJECT_ROOT"
echo "========================================="

case $MODE in
    verify)
        echo "使用 Dockerfile.verify 进行快速验证..."
        DOCKERFILE="docker/Dockerfile.verify"
        IMAGE_NAME="manipulator:verify"
        ;;
    build)
        echo "使用 Dockerfile.build 进行完整构建..."
        DOCKERFILE="docker/Dockerfile.build"
        IMAGE_NAME="manipulator:build"
        ;;
    *)
        echo "错误: 未知模式 '$MODE'"
        echo "用法: $0 [verify|build]"
        exit 1
        ;;
esac

echo ""
echo "步骤 1: 构建 Docker 镜像..."
docker build -f "$DOCKERFILE" -t "$IMAGE_NAME" "$PROJECT_ROOT"

if [ $? -ne 0 ]; then
    echo "错误: Docker 镜像构建失败"
    exit 1
fi

echo ""
echo "步骤 2: 运行容器验证编译..."
docker run --rm "$IMAGE_NAME"

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✅ 编译验证成功！"
    echo "========================================="
else
    echo ""
    echo "========================================="
    echo "❌ 编译验证失败"
    echo "========================================="
    exit 1
fi
