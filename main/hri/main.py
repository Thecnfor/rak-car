#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Robot Face HRI Application
使用PySide2显示QML机器人面部界面
需要在Jetson设备上运行，启动前需要配置X服务器
"""

import sys
import os
from PySide2.QtCore import QUrl
from PySide2.QtGui import QGuiApplication
from PySide2.QtQml import QQmlApplicationEngine

def setup_display_environment():
    """配置 Jetson 显示与 Qt 环境以通过 DP 输出运行"""
    # 指向本地 X11 显示会话
    os.environ.setdefault('DISPLAY', ':0')
    # 强制使用 XCB 后端，避免 Wayland/GL 集成问题
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    os.environ.setdefault('QT_XCB_GL_INTEGRATION', 'none')
    # 使用软件渲染以提升稳定性（Jetson 上部分环境的 OpenGL 会报错）
    os.environ.setdefault('QT_OPENGL', 'software')
    os.environ.setdefault('QT_QUICK_BACKEND', 'software')

def run_qml_app():
    """运行QML应用"""
    # 首先配置显示环境
    setup_display_environment()
    
    app = QGuiApplication(sys.argv)
    
    # 创建QML引擎
    engine = QQmlApplicationEngine()
    
    # 加载QML文件
    qml_file = os.path.join(os.path.dirname(__file__), "RobotFace.qml")
    engine.load(QUrl.fromLocalFile(qml_file))
    
    if not engine.rootObjects():
        return -1
    
    return app.exec_()


if __name__ == "__main__":
    sys.exit(run_qml_app())