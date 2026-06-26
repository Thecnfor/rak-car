import QtQuick 2.12

Item {
    id: eye
    width: 120
    height: 120

    // 眼睛状态属性
    property real pupilX: 0  // 瞳孔X偏移 (-1 到 1)
    property real pupilY: 0  // 瞳孔Y偏移 (-1 到 1)
    property real blinkAmount: 0  // 眨眼程度 (0=睁开, 1=闭合)
    property real eyeScale: 1.0  // 眼睛缩放
    property color eyeColor: "#E8F4FD"  // 眼睛颜色
    property real glowIntensity: 0.8  // 发光强度

    // 眼睛外轮廓
    Rectangle {
        id: eyeOuter
        anchors.centerIn: parent
        width: parent.width * eye.eyeScale
        height: parent.height * eye.eyeScale * (1 - eye.blinkAmount * 0.9)
        radius: width / 2
        color: "transparent"
        border.color: eye.eyeColor
        border.width: 3

        // 外发光效果 (简化版本)
        Rectangle {
            anchors.fill: eyeOuter
            anchors.margins: -10
            color: "transparent"
            border.color: Qt.rgba(eye.eyeColor.r, eye.eyeColor.g, eye.eyeColor.b, eye.glowIntensity * 0.3)
            border.width: 8
            radius: width / 2
            opacity: eye.glowIntensity * 0.5
        }

        // 眼睛内部填充
        Rectangle {
            id: eyeInner
            anchors.centerIn: parent
            width: parent.width * 0.85
            height: parent.height * 0.85
            radius: width / 2
            color: Qt.rgba(eye.eyeColor.r, eye.eyeColor.g, eye.eyeColor.b, 0.3)

            // 瞳孔
            Rectangle {
                id: pupil
                width: parent.width * 0.4
                height: parent.height * 0.4
                radius: width / 2
                color: eye.eyeColor

                // 瞳孔位置计算
                x: parent.width / 2 - width / 2 + eye.pupilX * (parent.width * 0.2)
                y: parent.height / 2 - height / 2 + eye.pupilY * (parent.height * 0.2)

                // 瞳孔发光 (简化版本)
                Rectangle {
                    anchors.fill: pupil
                    anchors.margins: -4
                    color: "transparent"
                    border.color: Qt.rgba(eye.eyeColor.r, eye.eyeColor.g, eye.eyeColor.b, eye.glowIntensity * 0.4)
                    border.width: 4
                    radius: width / 2
                    opacity: eye.glowIntensity * 0.6
                }

                // 瞳孔高光
                Rectangle {
                    width: parent.width * 0.3
                    height: parent.height * 0.3
                    radius: width / 2
                    color: Qt.rgba(1, 1, 1, 0.8)
                    x: parent.width * 0.2
                    y: parent.height * 0.2
                }
            }
        }
    }

    // 工作状态动画 - 轻微的缩放脉冲
    SequentialAnimation {
        id: workingAnimation
        loops: Animation.Infinite
        NumberAnimation {
            target: eye
            property: "eyeScale"
            to: 1.08
            duration: 800
            easing.type: Easing.InOutSine
        }
        NumberAnimation {
            target: eye
            property: "eyeScale"
            to: 1.02
            duration: 800
            easing.type: Easing.InOutSine
        }
    }

    // 成功状态动画 - 快速闪烁
    SequentialAnimation {
        id: successAnimation
        loops: 3
        NumberAnimation {
            target: eye
            property: "glowIntensity"
            to: 1.8
            duration: 150
        }
        NumberAnimation {
            target: eye
            property: "glowIntensity"
            to: 1.0
            duration: 150
        }
    }

    // 错误状态动画 - 快速红色闪烁
    SequentialAnimation {
        id: errorAnimation
        loops: 4
        NumberAnimation {
            target: eye
            property: "glowIntensity"
            to: 2.0
            duration: 100
        }
        NumberAnimation {
            target: eye
            property: "glowIntensity"
            to: 0.8
            duration: 100
        }
    }

    // 思考状态动画 - 慢速左右移动瞳孔
    SequentialAnimation {
        id: thinkingAnimation
        loops: Animation.Infinite
        NumberAnimation {
            target: eye
            property: "pupilX"
            to: eye.pupilX + 5
            duration: 1500
            easing.type: Easing.InOutQuad
        }
        NumberAnimation {
            target: eye
            property: "pupilX"
            to: eye.pupilX - 5
            duration: 1500
            easing.type: Easing.InOutQuad
        }
        NumberAnimation {
            target: eye
            property: "pupilX"
            to: eye.pupilX
            duration: 1500
            easing.type: Easing.InOutQuad
        }
    }

    // 自动眨眼动画
    SequentialAnimation {
        id: autoBlinkAnimation
        running: true
        loops: Animation.Infinite

        PauseAnimation {
            duration: Math.random() * 3000 + 2000
        }  // 2-5秒随机间隔

        SequentialAnimation {
            NumberAnimation {
                target: eye
                property: "blinkAmount"
                to: 1.0
                duration: 150
                easing.type: Easing.OutQuad
            }
            NumberAnimation {
                target: eye
                property: "blinkAmount"
                to: 0.0
                duration: 150
                easing.type: Easing.InQuad
            }
        }
    }

    // 瞳孔移动动画 - 现在由外部控制，不再自动运行
    ParallelAnimation {
        id: pupilMovementAnimation
        NumberAnimation {
            id: pupilXAnimation
            target: eye
            property: "pupilX"
            duration: 800
            easing.type: Easing.InOutQuad
        }
        NumberAnimation {
            id: pupilYAnimation
            target: eye
            property: "pupilY"
            duration: 800
            easing.type: Easing.InOutQuad
        }
    }

    // 微动画 - 添加细微的自然摆动
    SequentialAnimation {
        id: microMovementAnimation
        running: true
        loops: Animation.Infinite

        PauseAnimation {
            duration: Math.random() * 800 + 400  // 更频繁的微动
        }

        ParallelAnimation {
            NumberAnimation {
                target: eye
                property: "pupilX"
                to: eye.pupilX + (Math.random() - 0.5) * 0.05  // 更细微的移动
                duration: 200 + Math.random() * 200
                easing.type: Easing.InOutSine
            }
            NumberAnimation {
                target: eye
                property: "pupilY"
                to: eye.pupilY + (Math.random() - 0.5) * 0.04
                duration: 200 + Math.random() * 200
                easing.type: Easing.InOutSine
            }
        }
    }

    // 注视行为 - 偶尔会有短暂的专注凝视
    Timer {
        id: gazeTimer
        interval: 8000 + Math.random() * 5000  // 8-13秒随机间隔
        running: true
        repeat: true
        onTriggered: {
            // 停止微动画，进行短暂凝视
            microMovementAnimation.stop()
            
            // 凝视持续时间
            gazeHoldTimer.start()
            
            // 重新设置随机间隔
            interval = 8000 + Math.random() * 5000
        }
    }

    Timer {
        id: gazeHoldTimer
        interval: 1000 + Math.random() * 2000  // 1-3秒凝视时间
        onTriggered: {
            // 恢复微动画
            microMovementAnimation.start()
        }
    }

    // 公共方法：设置情感状态
    function setEmotion(emotion) {
        switch (emotion) {
        case "happy":
            eyeScale = 1.1;
            glowIntensity = 1.0;
            eyeColor = "#B8E6B8";  // 淡绿色
            break;
        case "sleepy":
            eyeScale = 0.8;
            glowIntensity = 0.4;
            blinkAmount = 0.3;
            eyeColor = "#FFE4B5";  // 暖黄色
            break;
        case "surprised":
            eyeScale = 1.3;
            glowIntensity = 1.2;
            eyeColor = "#FFB6C1";  // 淡粉色
            break;
        case "working":
            eyeScale = 1.05;
            glowIntensity = 1.1;
            eyeColor = "#87CEEB";  // 天蓝色，表示专注工作
            // 启动工作状态的特殊动画
            workingAnimation.start();
            break;
        case "success":
            eyeScale = 1.2;
            glowIntensity = 1.3;
            eyeColor = "#90EE90";  // 亮绿色，表示成功
            // 启动成功闪烁动画
            successAnimation.start();
            break;
        case "error":
            eyeScale = 1.1;
            glowIntensity = 1.4;
            eyeColor = "#FFB6C1";  // 淡红色，表示错误
            // 启动错误闪烁动画
            errorAnimation.start();
            break;
        case "thinking":
            eyeScale = 0.95;
            glowIntensity = 0.9;
            eyeColor = "#DDA0DD";  // 淡紫色，表示思考
            // 启动思考动画
            thinkingAnimation.start();
            break;
        case "idle":
        default:
            eyeScale = 1.0;
            glowIntensity = 0.8;
            blinkAmount = 0.0;
            eyeColor = "#E8F4FD";  // 淡蓝白色
            // 停止所有特殊动画
            workingAnimation.stop();
            successAnimation.stop();
            errorAnimation.stop();
            thinkingAnimation.stop();
            break;
        }
    }

    // 公共方法：手动眨眼
    function blink() {
        manualBlinkAnimation.start();
    }

    // 手动眨眼动画
    SequentialAnimation {
        id: manualBlinkAnimation
        NumberAnimation {
            target: eye
            property: "blinkAmount"
            to: 1.0
            duration: 100
        }
        NumberAnimation {
            target: eye
            property: "blinkAmount"
            to: 0.0
            duration: 100
        }
    }

    // 公共方法：看向指定方向
    function lookAt(x, y) {
        microMovementAnimation.stop();
        gazeTimer.stop();
        gazeHoldTimer.stop();
        pupilXAnimation.to = Math.max(-1, Math.min(1, x));
        pupilYAnimation.to = Math.max(-1, Math.min(1, y));
        pupilMovementAnimation.start();
    }

    // 公共方法：平滑移动到指定位置
    function moveTo(x, y, duration) {
        microMovementAnimation.stop();
        gazeTimer.stop();
        gazeHoldTimer.stop();
        pupilXAnimation.duration = duration || 800;
        pupilYAnimation.duration = duration || 800;
        pupilXAnimation.to = Math.max(-1, Math.min(1, x));
        pupilYAnimation.to = Math.max(-1, Math.min(1, y));
        pupilMovementAnimation.start();
    }

    // 公共方法：恢复微动画和注视行为
    function resumeMicroMovement() {
        microMovementAnimation.start();
        gazeTimer.start();
    }

    // 瞳孔目标动画 - 移除，已被新的动画系统替代
}
