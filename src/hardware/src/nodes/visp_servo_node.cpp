// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// visp_servo_node — 机械臂视觉伺服 (ViSP, eye-to-hand, 2D 图像居中).
//
// 架构 (完整链路, 安全门/控制器永远在环内):
//   camera image + camera_info + DetectionArray
//     → 本节点 (ViSP vpServo 计算相机系速度)
//     → /rak/control/arm/servo_twist (TwistStamped, base_link 系)
//     → moveit_servo (move_group.launch.py) → joint_trajectory_controller
//     → ros2_control → MC602 bridge → MC602
//
// 物理约束 (为什么是 eye-to-hand + 图像居中, 而不是度量的 3D 伺服):
//   1. 机械臂相机 (camera_arm_optical) 固定在 base_link 上 —— 不随末端移动,
//      是 eye-to-hand 结构。此时能改变图像特征的是"被抓持在末端上的物体"
//      (臂动 → 物体在固定相机视场中移动), 不是静止的桌面上物体。
//   2. DetectionArray 只有 2D 包围盒 + 无深度/目标尺寸/位姿 —— 没有真实深度
//      契约, 禁止虚构 Z 做度量伺服。
//   因此控制律是**图像居中 IBVS**: 把检测质心归一化坐标驱到图像中心。
//       v_c = -lambda * L^+ * (s - s*)
//   特征 Z 取单位深度 1.0 (ViSP 官方对无深度 2D 特征的标准做法, 见
//   vpFeaturePoint buildFrom 例程 Zd=1): 收敛到图像中心 (IBVS 对未知深度的
//   鲁棒性), 但**不声称度量精度**。要做度量 3D 伺服, 需要深度/靶标契约
//   (AprilTag 等), 那是后续扩展点。
//
// 安全策略 (任何一项不满足 → 发零速度, 绝不含糊):
//   - enabled 参数/服务关 → 零
//   - 无 camera_info / K 全零 (未标定) → 零 (永不虚构内参)
//   - 图像或检测流超时 (feature_timeout_sec) → 零
//   - 检测数组长度不一致 / 非有限 / 越界 / 尺寸不一致 → 零
//   - 无目标 (分数低于阈值 / 类别不匹配) → 零
//   - TF 查不到 camera→command 变换 → 零
//   - 死区: 质心距图像中心 < deadband_px → 零 (防抖)
//
// 指令归属 (同一时刻只有一个机械臂驱动器, 由任务层编排):
//   move_group (moveit) / arm_cartesian_move_node / arm_node / 本节点
//   互斥 —— 本节点只应在其被明确使能时运行 (SetBool 服务或 enabled 参数),
//   任务层负责在"视觉追踪"阶段独占。

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <msgs/msg/detection_array.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace
{

// Humble 的 ViSP 以 ENABLE_VISP_NAMESPACE=ON 构建, 全部类型在 visp:: 下
// (visp::vpMatrix, visp::vpServo, ...), 见 VISPConfig.cmake 的
// set(ENABLE_VISP_NAMESPACE "ON")。
using namespace visp;

// 6x6 velocity-twist matrix bVc: base-frame twist = bVc * camera-frame twist.
// Built from the pose of the camera in base (bMc), ViSP convention:
//   bVc = [ R      [t]x * R ]
//         [ 0          R    ]
// where R,t are the rotation/translation of the camera frame in base.
vpMatrix buildVelocityTwistBaseFromCamera(const vpTranslationVector & t,
                                          const vpRotationMatrix & R)
{
  // skew of t
  vpMatrix S(3, 3);
  S[0][0] = 0.0;      S[0][1] = -t[2];  S[0][2] = t[1];
  S[1][0] = t[2];     S[1][1] = 0.0;    S[1][2] = -t[0];
  S[2][0] = -t[1];    S[2][1] = t[0];   S[2][2] = 0.0;

  vpMatrix SR = S * vpMatrix(R);  // [t]x * R

  vpMatrix bVc(6, 6);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      bVc[i][j] = R[i][j];          // top-left: R
      bVc[i][3 + j] = SR[i][j];     // top-right: [t]x * R
      bVc[3 + i][j] = 0.0;          // bottom-left: 0
      bVc[3 + i][3 + j] = R[i][j];  // bottom-right: R
    }
  }
  return bVc;
}

class VispServoNode : public rclcpp::Node
{
public:
  VispServoNode()
  : Node("visp_servo_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- 参数 ----------------------------------------------------------
    image_topic_ = declare_parameter<std::string>("image_topic",
      "/rak/sensors/camera/arm/image_compressed");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic",
      "/rak/sensors/camera/arm/camera_info");
    target_topic_ = declare_parameter<std::string>("target_topic",
      "/rak/perception/detections/task");
    servo_topic_ = declare_parameter<std::string>("servo_topic",
      "/rak/control/arm/servo_twist");
    camera_frame_ = declare_parameter<std::string>("camera_frame",
      "camera_arm_optical");
    command_frame_ = declare_parameter<std::string>("command_frame",
      "base_link");
    target_class_ = declare_parameter<std::string>("target_class", "");
    score_threshold_ = declare_parameter<double>("score_threshold", 0.5);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    feature_timeout_sec_ = declare_parameter<double>("feature_timeout_sec", 0.5);
    linear_gain_ = declare_parameter<double>("linear_gain", 0.5);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 0.05);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.2);
    deadband_px_ = declare_parameter<double>("deadband_px", 5.0);
    enabled_ = declare_parameter<bool>("enabled", false);

    // ---- 订阅 ----------------------------------------------------------
    img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
        last_image_stamp_ = m->header.stamp;
      });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        cam_info_ = m;
      });
    det_sub_ = create_subscription<msgs::msg::DetectionArray>(
      target_topic_, rclcpp::QoS(5),
      [this](msgs::msg::DetectionArray::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        last_det_ = m;
      });

    // ---- 发布 ----------------------------------------------------------
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      servo_topic_, rclcpp::QoS(5).reliable());
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "/rak/control/arm/visp_status", rclcpp::QoS(1).transient_local());

    // ---- 使能服务 ------------------------------------------------------
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_enabled",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        {
          std::lock_guard<std::mutex> lk(mu_);
          enabled_ = req->data;
        }
        res->success = true;
        res->message = enabled_ ? "visual servo enabled" : "visual servo disabled";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    // ---- 伺服任务 (初始化一次, 每 tick 更新特征) ------------------------
    task_.setServo(vpServo::EYEINHAND_CAMERA);
    task_.setInteractionMatrixType(vpServo::CURRENT);
    // 反演方法默认 PSEUDO_INVERSE, Humble 的 vpServo 没有 setInversion 接口。
    task_.setLambda(linear_gain_);
    s_star_.buildFrom(0.0, 0.0, 1.0);   // 期望 = 图像中心 (归一化)
    task_.addFeature(s_, s_star_);      // vpFeaturePoint: 选择 x,y 两维

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      [this]() { tick(); });
  }

  ~VispServoNode() override
  {
    task_.kill();
  }

private:
  // 报告当前门控原因 (状态机去重, 避免刷屏)。
  void report_gate(const std::string & reason, bool servoing)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (reason == last_gate_) {
      return;
    }
    last_gate_ = reason;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = servoing ? diagnostic_msgs::msg::DiagnosticStatus::OK
                        : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.name = "visp_servo";
    st.message = reason;
    status_pub_->publish(st);
    if (servoing) {
      RCLCPP_INFO(get_logger(), "servoing: %s", reason.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "gate: %s", reason.c_str());
    }
  }

  void publish_zero(const std::string & reason)
  {
    report_gate(reason, false);
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = now();
    zero.header.frame_id = command_frame_;
    twist_pub_->publish(zero);
  }

  // 从检测数组选目标: 空 target_class → 最高分; 否则第一个匹配类别且过阈值。
  bool pick_target(const msgs::msg::DetectionArray & det, size_t & idx) const
  {
    const size_t n = det.scores.size();
    if (n == 0) {
      return false;
    }
    if (target_class_.empty()) {
      size_t best = 0;
      for (size_t i = 1; i < n; ++i) {
        if (det.scores[i] > det.scores[best]) {
          best = i;
        }
      }
      if (det.scores[best] < score_threshold_) {
        return false;
      }
      idx = best;
      return true;
    }
    for (size_t i = 0; i < n; ++i) {
      if (det.scores[i] >= score_threshold_ &&
          i < det.class_names.size() && det.class_names[i] == target_class_) {
        idx = i;
        return true;
      }
    }
    return false;
  }

  void tick()
  {
    // ---- 使能 ----------------------------------------------------------
    if (!enabled_) {
      publish_zero("disabled");
      return;
    }

    // ---- 快照状态 (回调线程 → 定时器线程, 一次加锁) ---------------------
    msgs::msg::DetectionArray::ConstSharedPtr det;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    rclcpp::Time last_image_stamp;
    {
      std::lock_guard<std::mutex> lk(mu_);
      det = last_det_;
      info = cam_info_;
      last_image_stamp = last_image_stamp_;
    }
    if (!info) {
      publish_zero("no camera_info yet");
      return;
    }
    const double fx = info->k[0], fy = info->k[4];
    const double cx = info->k[2], cy = info->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      publish_zero("camera not calibrated (zero K)");
      return;
    }

    // ---- 新鲜度 --------------------------------------------------------
    const auto now_t = now();
    if (!det) {
      publish_zero("no detections yet");
      return;
    }
    const rclcpp::Time det_stamp(det->header.stamp);
    if (det_stamp.seconds() <= 0.0) {
      publish_zero("no detections yet");
      return;
    }
    if (last_image_stamp.seconds() <= 0.0) {
      publish_zero("no image yet");
      return;
    }
    if ((now_t - last_image_stamp).seconds() > feature_timeout_sec_) {
      publish_zero("image stream stale");
      return;
    }
    if ((now_t - det_stamp).seconds() > feature_timeout_sec_) {
      publish_zero("detection stream stale");
      return;
    }

    // ---- 数组一致性 ----------------------------------------------------
    const size_t n = det->scores.size();
    if (det->xs.size() != n || det->ys.size() != n ||
        det->widths.size() != n || det->heights.size() != n ||
        det->class_ids.size() != n) {
      publish_zero("detection array length mismatch");
      return;
    }
    if (det->image_width == 0 || det->image_height == 0 ||
        static_cast<uint32_t>(info->width) != det->image_width ||
        static_cast<uint32_t>(info->height) != det->image_height) {
      publish_zero("detection/camera image size mismatch");
      return;
    }

    // ---- 选目标 --------------------------------------------------------
    size_t idx = 0;
    if (!pick_target(*det, idx)) {
      publish_zero("no target above threshold");
      return;
    }
    const double u0 = det->xs[idx], v0 = det->ys[idx];
    const double w = det->widths[idx], h = det->heights[idx];
    if (!std::isfinite(u0) || !std::isfinite(v0) ||
        !std::isfinite(w) || !std::isfinite(h) || w <= 0.0 || h <= 0.0) {
      publish_zero("invalid (non-finite/degenerate) bbox");
      return;
    }

    // ---- TF: camera → command -----------------------------------------
    vpRotationMatrix R;
    vpTranslationVector t;
    try {
      const auto ts = tf_buffer_.lookupTransform(
        command_frame_, camera_frame_, tf2::TimePointZero);
      const auto & q = ts.transform.rotation;
      const auto & tr = ts.transform.translation;
      R = vpRotationMatrix(vpQuaternionVector(q.x, q.y, q.z, q.w));
      t = vpTranslationVector(tr.x, tr.y, tr.z);
    } catch (const tf2::TransformException & e) {
      publish_zero(std::string("tf lookup failed: ") + e.what());
      return;
    }

    // ---- 质心 + 死区 ---------------------------------------------------
    const double u = u0 + w / 2.0;
    const double v = v0 + h / 2.0;
    const double px_err = std::hypot(u - cx, v - cy);
    if (px_err < deadband_px_) {
      publish_zero("within deadband (centered)");
      return;
    }

    // ---- ViSP 控制律 (单位深度 Z=1, 图像居中) -------------------------
    const double x = (u - cx) / fx;
    const double y = (v - cy) / fy;
    s_.buildFrom(x, y, 1.0);
    const vpColVector v_cam = task_.computeControlLaw();  // 6x1: vx,vy,vz,wx,wy,wz

    // ---- 相机速度 → base 速度 ----------------------------------------
    const vpMatrix bVc = buildVelocityTwistBaseFromCamera(t, R);
    const vpColVector v_base = bVc * v_cam;

    // ---- 限幅 ----------------------------------------------------------
    double vx = v_base[0], vy = v_base[1], vz = v_base[2];
    double wx = v_base[3], wy = v_base[4], wz = v_base[5];
    const double lin = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (lin > max_linear_velocity_) {
      const double s = max_linear_velocity_ / lin;
      vx *= s; vy *= s; vz *= s;
    }
    const double ang = std::sqrt(wx * wx + wy * wy + wz * wz);
    if (ang > max_angular_velocity_) {
      const double s = max_angular_velocity_ / ang;
      wx *= s; wy *= s; wz *= s;
    }

    // ---- 发布 ----------------------------------------------------------
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now_t;
    cmd.header.frame_id = command_frame_;
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.linear.z = vz;
    cmd.twist.angular.x = wx;
    cmd.twist.angular.y = wy;
    cmd.twist.angular.z = wz;
    twist_pub_->publish(cmd);

    report_gate("servoing (unit-depth image centering)", true);
  }

  // ---- 参数 ------------------------------------------------------------
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string target_topic_;
  std::string servo_topic_;
  std::string camera_frame_;
  std::string command_frame_;
  std::string target_class_;
  double score_threshold_{0.5};
  double publish_rate_hz_{30.0};
  double feature_timeout_sec_{0.5};
  double linear_gain_{0.5};
  double max_linear_velocity_{0.05};
  double max_angular_velocity_{0.2};
  double deadband_px_{5.0};
  bool enabled_{false};

  // ---- 订阅/发布 -------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<msgs::msg::DetectionArray>::SharedPtr det_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 状态 (回调线程 → 定时器线程, 用互斥保护) ----------------------
  std::mutex mu_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info_;
  msgs::msg::DetectionArray::ConstSharedPtr last_det_;
  rclcpp::Time last_image_stamp_{0, 0, RCL_ROS_TIME};
  std::string last_gate_;

  // ---- TF --------------------------------------------------------------
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ---- ViSP (EYEINHAND_CAMERA: v_c = -lambda * L^+ * e) ---------------
  vpServo task_;
  vpFeaturePoint s_;
  vpFeaturePoint s_star_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VispServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
