#!/bin/bash
set -euo pipefail

# HRI 底层直接输出启动脚本 - 不依赖 GDM 桌面环境
# 适用于 Jetson Orin Nano Ubuntu 20.04 L4T

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_ENTRY="$APP_DIR/main.py"

echo "[HRI] Starting HRI service with direct output..."

# 优先使用 EGLFS (DRM/KMS) 直连渲染，无需 X/GDM
if [ -e /dev/dri/card0 ] && [ -f /usr/lib/aarch64-linux-gnu/qt5/plugins/platforms/libqeglfs.so ]; then
    echo "[HRI] Using EGLFS (DRM/KMS) direct output"
    export QT_QPA_PLATFORM=eglfs
    export QT_QPA_EGLFS_HIDECURSOR=1
    export QT_LOGGING_RULES="*=false"
    export QT_SCALE_FACTOR=1
    export QT_AUTO_SCREEN_SCALE_FACTOR=0
    exec /usr/bin/python3 "$APP_ENTRY"
fi

# 等待系统完全启动
sleep 5

# 检查是否已有 X 服务器运行
if ! DISPLAY=:0 xdpyinfo >/dev/null 2>&1; then
    echo "[HRI] No X server found, trying to start one..."
    
    # 尝试启动用户级 X 服务器（Xvfb 虚拟显示）
    if command -v Xvfb >/dev/null 2>&1; then
        echo "[HRI] Starting Xvfb virtual display..."
        Xvfb :0 -screen 0 1024x600x24 -ac -nolisten tcp &
        X_SERVER_PID=$!
        
        # 等待 Xvfb 启动
        echo "[HRI] Waiting for Xvfb to start..."
        for i in {1..15}; do
            if DISPLAY=:0 xdpyinfo >/dev/null 2>&1; then
                echo "[HRI] Xvfb is ready"
                break
            fi
            sleep 1
        done
    else
        echo "[HRI] Xvfb not available, trying to use existing display..."
    fi
    
    # 如果还是没有 X 服务器，尝试等待系统 X 服务器
    if ! DISPLAY=:0 xdpyinfo >/dev/null 2>&1; then
        echo "[HRI] Waiting for system X server..."
        for i in {1..60}; do
            if DISPLAY=:0 xdpyinfo >/dev/null 2>&1; then
                echo "[HRI] System X server is ready"
                break
            fi
            sleep 1
        done
    fi
    
    # 最后检查
    if ! DISPLAY=:0 xdpyinfo >/dev/null 2>&1; then
        echo "[HRI] WARNING: No X server available, trying framebuffer mode..."
        export QT_QPA_PLATFORM=linuxfb
        export QT_QPA_FB_DEVICE=/dev/fb0
    fi
else
    echo "[HRI] Using existing X server"
fi

# 如果有 X 服务器，配置显示输出
if DISPLAY=:0 xdpyinfo >/dev/null 2>&1; then
    echo "[HRI] Configuring display output..."
    DISPLAY=:0 xrandr --output DP-1 --mode 1024x600 --primary 2>/dev/null || echo "[HRI] Warning: Could not set DP-1 mode"
    
    # 设置屏幕节能配置
    echo "[HRI] Configuring display power management..."
    DISPLAY=:0 xset -dpms 2>/dev/null || echo "[HRI] Warning: Could not disable DPMS"
    DISPLAY=:0 xset s off 2>/dev/null || echo "[HRI] Warning: Could not disable screensaver"
    
    # 设置环境变量，优先使用底层渲染
    export DISPLAY=:0
    export QT_QPA_PLATFORM=xcb
    export QT_XCB_GL_INTEGRATION=none
    export QT_OPENGL=software
    export QT_QUICK_BACKEND=software
else
    echo "[HRI] Using framebuffer mode"
    export QT_QPA_PLATFORM=linuxfb
    export QT_QPA_FB_DEVICE=/dev/fb0
    export QT_QPA_FB_SIZE=1024x600
fi

# 通用环境变量
export QT_LOGGING_RULES="*=false"  # 减少日志输出
export QT_SCALE_FACTOR=1
export QT_AUTO_SCREEN_SCALE_FACTOR=0

echo "[HRI] Starting PyQt application: $APP_ENTRY"
exec /usr/bin/python3 "$APP_ENTRY"