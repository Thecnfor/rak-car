import QtQuick 2.12
import QtQuick.Window 2.12

Window {
    id: robotFace

    // 面部状态属性
    property string currentEmotion: "idle"
    property bool isActive: true
    
    // 响应式设计属性
    property real aspectRatio: width / height
    property real baseAspectRatio: 1024 / 600
    property real widthScale: width / 1024
    property real heightScale: height / 600
    property real uniformScale: Math.min(widthScale, heightScale)
    property real aspectAdjustment: aspectRatio > baseAspectRatio ? Math.min(1.2, 1 + (aspectRatio - baseAspectRatio) * 0.5) : Math.max(0.8, 1 - (baseAspectRatio - aspectRatio) * 0.3)
    property real scaleFactor: uniformScale * aspectAdjustment
    
    // 基于人体工学的眼睛设计参数
    property real humanPupilDistance: 62
    property real screenPPI: 96
    property real mmToPixel: screenPPI / 25.4
    
    // 动态瞳距调整参数
    property string ageGroup: "adult"
    property string gender: "mixed"
    property real customPupilDistance: 62
    property bool useCustomPupilDistance: false
    
    // 不同人群的瞳距范围
    property var pupilDistanceRanges: {
        "adult_male": {"min": 60, "max": 73, "avg": 62},
        "adult_female": {"min": 53, "max": 68, "avg": 58},
        "adult_mixed": {"min": 55, "max": 75, "avg": 62},
        "child": {"min": 40, "max": 58, "avg": 50},
        "elderly": {"min": 58, "max": 70, "avg": 64}
    }
    
    // 计算当前应使用的瞳距
    function getCurrentPupilDistance() {
        if (useCustomPupilDistance) {
            return Math.max(40, Math.min(80, customPupilDistance))
        }
        
        var key = ageGroup === "adult" ? "adult_" + gender : ageGroup
        if (pupilDistanceRanges[key]) {
            return pupilDistanceRanges[key].avg
        }
        return 62
    }
    
    property real currentPupilDistance: getCurrentPupilDistance()
    
    // 响应式眼睛尺寸计算 - 针对7寸屏幕小幅优化
    property real baseEyeSize: Math.min(width * 0.22, height * 0.35)  // 再减小基础尺寸比例
    property real eyeSize: Math.max(130, Math.min(360, baseEyeSize * scaleFactor))  // 再降低最小和最大尺寸
    
    // 基于人体工学的眼睛间距计算 - 再加大间距
    property real baseEyeSpacing: Math.min(currentPupilDistance * mmToPixel * scaleFactor, eyeSize * 0.9)  // 进一步放宽最大间距限制
    property real eyeSpacing: Math.max(eyeSize * 0.4, Math.min(eyeSize * 0.9, baseEyeSpacing))  // 再扩大间距范围，让眼睛更分开
    
    width: 1024
    height: 600
    minimumWidth: 480
    minimumHeight: 360
    visible: true
    visibility: Window.FullScreen
    title: "Robot Face"
    color: "#0A0A0A"

    // 主面部容器
    Rectangle {
        id: faceContainer
        anchors.fill: parent
        color: "transparent"

    // 眼睛容器 - 针对7寸屏幕优化
    Item {
        id: eyeContainer
        width: eyeSpacing + eyeSize * 2
        height: eyeSize
        anchors.centerIn: parent

        // 左眼
        Eye {
            id: leftEye
            width: robotFace.eyeSize
            height: robotFace.eyeSize
            x: 0
            anchors.verticalCenter: parent.verticalCenter
        }

        // 右眼
        Eye {
            id: rightEye
            width: robotFace.eyeSize
            height: robotFace.eyeSize
            x: robotFace.eyeSize + robotFace.eyeSpacing
            anchors.verticalCenter: parent.verticalCenter
        }
        }
    }

    // 瞳距调整函数
    function setPupilDistance(ageGroup, gender) {
        robotFace.ageGroup = ageGroup
        robotFace.gender = gender
        robotFace.useCustomPupilDistance = false
        robotFace.currentPupilDistance = robotFace.getCurrentPupilDistance()
        console.log("瞳距已调整为:", robotFace.currentPupilDistance + "mm", "年龄组:", ageGroup, "性别:", gender)
    }
    
    function setCustomPupilDistance(distance) {
        robotFace.customPupilDistance = distance
        robotFace.useCustomPupilDistance = true
        robotFace.currentPupilDistance = robotFace.getCurrentPupilDistance()
        console.log("自定义瞳距已设置为:", distance + "mm")
    }
    
    function resetPupilDistance() {
        robotFace.ageGroup = "adult"
        robotFace.gender = "mixed"
        robotFace.useCustomPupilDistance = false
        robotFace.currentPupilDistance = robotFace.getCurrentPupilDistance()
        console.log("瞳距已重置为默认值:", robotFace.currentPupilDistance + "mm")
    }

    // 眼睛协调控制系统
    property real targetPupilX: 0
    property real targetPupilY: 0
    property bool eyesLocked: true  // 眼睛是否同步移动

    // 眼睛协调移动定时器
    Timer {
        id: eyeCoordinationTimer
        interval: 2000 + Math.random() * 3000  // 2-5秒随机间隔
        running: true
        repeat: true
        onTriggered: {
            if (eyesLocked) {
                // 生成新的目标位置
                targetPupilX = (Math.random() - 0.5) * 1.2
                targetPupilY = (Math.random() - 0.5) * 0.8
                
                // 同步移动两个眼睛
                leftEye.moveTo(targetPupilX, targetPupilY, 1000 + Math.random() * 500)
                rightEye.moveTo(targetPupilX, targetPupilY, 1000 + Math.random() * 500)
            }
            
            // 重新设置随机间隔
            interval = 2000 + Math.random() * 3000
        }
    }

    // 公共方法：让两个眼睛看向指定方向
    function lookAt(x, y) {
        targetPupilX = x
        targetPupilY = y
        leftEye.lookAt(x, y)
        rightEye.lookAt(x, y)
    }

    // 公共方法：设置眼睛同步状态
    function setEyesLocked(locked) {
        eyesLocked = locked
        if (locked) {
            // 如果锁定，让两个眼睛移动到相同位置
            rightEye.moveTo(leftEye.pupilX, leftEye.pupilY)
        }
    }

    // 公共方法：设置情感状态
    function setEmotion(emotion) {
        leftEye.setEmotion(emotion)
        rightEye.setEmotion(emotion)
        currentEmotion = emotion
    }
}