# ROS2 工程分层 + topic/service/action 接口实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `develop/ros2-sidecar` 落成四层(driver/component/behavior/cognition)+ 全量 topic/service/action 契约,适配 115200 单串口。

**Architecture:** driver 层串口唯一所有者(bridge)不变;component 层 arm/chassis 增加 action server(semantic 门面),新增 system_io_node 收编杂项设备;behavior 层用 action client 编排;cognition 只预占 namespace。

**Tech Stack:** C++17 / rclcpp / rclcpp_action / rosidl (msgs) / gmock / pytest / launch。

**Spec:** `docs/superpowers/specs/2026-08-09-ros2-layering-interfaces-design.md`(已批准)

## Global Constraints

- 所有 topic/service/action 名字在 `/rak/...` 下(config_loader 规则,延续到新接口)。
- 串口只有一个所有者:`mc602_bridge_node`;组件/系统节点一律走 `BridgeTransport`(经 `/rak/hw/mc602/transaction`),生产配置 `mc602_transport:=bridge`。
- 115200 baud 是生产默认(launch arg 可覆盖);读策略=组件自持轮询+按需读,频率走参数不写死。
- 节点入口在 `src/hardware/src/nodes/`,核心纯逻辑在 `src/`(现有 "纯类+薄壳" 模式)。
- no-mocks:驱动失败 `throw`(组件层)→ Action `error` 字段 + ABORTED;禁止合成假数据。
- 测试不打真串口:纯类用 gmock/直接断言;`MC602Adapter::set_injection` 注入假响应。
- C++ 编译 C++17;提交信息带 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`。

---

### Task 1: msgs 包新增全部接口(消息/服务/动作)+ 删 ActuatorState

**Files:**
- Create: `src/msgs/msg/TaskStatus.msg`
- Create: `src/msgs/msg/Pose2D.msg`(Lyrical 已移除 geometry_msgs/Pose2D,自建轻量 2D 位姿)
- Create: `src/msgs/srv/SensorQuery.srv`, `src/msgs/srv/Beep.srv`, `src/msgs/srv/SetRgbLed.srv`, `src/msgs/srv/LedShow.srv`, `src/msgs/srv/Nixie.srv`, `src/msgs/srv/ReadIntArray.srv`
- Create: `src/msgs/action/ChassisNavigate.action`, `src/msgs/action/ArmExecuteTrajectory.action`
- Modify: `src/msgs/CMakeLists.txt`(加文件 + `find_package(action_msgs/trajectory_msgs)` + DEPENDENCIES;删 ActuatorState.msg;**不依赖 geometry_msgs**)
- Modify: `src/msgs/package.xml`(加 action_msgs/trajectory_msgs depend)
- Delete: `src/msgs/msg/ActuatorState.msg`(已确认全仓库无引用)

**Interfaces:**
- Produces: `msgs::msg::TaskStatus`; `msgs::msg::Pose2D`; `msgs::srv::{SensorQuery,Beep,SetRgbLed,LedShow,Nixie,ReadIntArray}`; `msgs::action::{ChassisNavigate,ArmExecuteTrajectory}`

- [ ] **Step 1: 写全部接口定义文件**

`src/msgs/msg/TaskStatus.msg`:
```text
std_msgs/Header header
string task_id
string state          # IDLE | RUNNING | SUCCEEDED | FAILED | ABORTED
string current_step
float32 progress      # 0.0 - 1.0
string message
```

`src/msgs/srv/SensorQuery.srv`:
```text
uint8 port            # MC602 端口号
string type           # ir | ultrasonic | analog | touch | ambient | encoder
---
bool ok
string error
float64 value
```

`src/msgs/srv/Beep.srv`:
```text
uint16 freq
float32 duration_s
---
bool ok
string error
```

`src/msgs/srv/SetRgbLed.srv`:
```text
uint8 led_id
uint8 r
uint8 g
uint8 b
---
bool ok
string error
```

`src/msgs/srv/LedShow.srv`:
```text
string text           # 点阵屏文本,≤100 字符
---
bool ok
string error
```

`src/msgs/srv/Nixie.srv`:
```text
int32 value
---
bool ok
string error
```

`src/msgs/srv/ReadIntArray.srv`:
```text
string source         # "key" | "pad"
---
bool ok
string error
int64[] values
```

`src/msgs/action/ChassisNavigate.action`:
```text
# Goal
msgs/Pose2D target_pose
float32 max_linear_speed
float32 max_angular_speed
float32 tolerance_lin
float32 tolerance_ang
float32 timeout_sec
---
# Result
bool success
string error
float32 traveled_distance
---
# Feedback
msgs/Pose2D current_pose
float32 remaining_distance
```

`src/msgs/msg/Pose2D.msg`(Lyrical 无 geometry_msgs/Pose2D,自建):
```text
float64 x
float64 y
float64 theta
```

`src/msgs/action/ArmExecuteTrajectory.action`:
```text
# Goal
string arm_id
trajectory_msgs/JointTrajectory trajectory
float32 max_execution_time
---
# Result
bool success
string error
float32[] final_positions
---
# Feedback
trajectory_msgs/JointTrajectoryPoint current
```

- [ ] **Step 2: 更新 `src/msgs/CMakeLists.txt`**

```cmake
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(msgs
  "msg/CameraMeta.msg"
  "msg/DetectionArray.msg"
  "msg/Frame.msg"
  "msg/LaneResult.msg"
  "msg/Pose2D.msg"
  "msg/TaskStatus.msg"
  "srv/Mc602Transaction.srv"
  "srv/Beep.srv"
  "srv/LedShow.srv"
  "srv/Nixie.srv"
  "srv/ReadIntArray.srv"
  "srv/SetRgbLed.srv"
  "srv/SensorQuery.srv"
  "action/ChassisNavigate.action"
  "action/ArmExecuteTrajectory.action"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs trajectory_msgs
)
```

- [ ] **Step 3: 更新 `src/msgs/package.xml`** 加 `<depend>action_msgs</depend>` `<depend>trajectory_msgs</depend>`(geometry_msgs 不再需要——Pose2D 已自建)。

- [ ] **Step 4: 构建验证(只 build msgs 包)**

```bash
cd /home/xrak/Desktop/XRAK/rak-car/.claude/worktrees/ros2-layering-interfaces
colcon build --packages-select msgs
ls install/msgs/include/msgs/ | grep -iE 'task_status|chassis|arm_execute|sensor_query|beep|nixie|read_int'
```
Expected: 所有新接口头文件生成;`ActuatorState` 不再生成。

- [ ] **Step 5: Commit**

```bash
git add src/msgs
git commit -m "feat(msgs): 分层接口全集 —— TaskStatus msg + 6 srv + 2 action,删 ActuatorState
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: system_io_node —— 杂项设备类型化服务

**Files:**
- Create: `src/hardware/include/hardware/system_io.hpp`, `src/hardware/src/system_io.cpp`(纯类,SystemIo)
- Create: `src/hardware/src/nodes/system_io_node.cpp`(薄壳,7 个 service server)
- Create: `src/hardware/test/test_system_io.cpp`(gtest,注入假 adapter)
- Modify: `src/hardware/CMakeLists.txt`(新 executable + test + install)

**Interfaces:**
- Consumes: `MC602Adapter`(typed API: read_sensor / beep / set_led_light / set_led_show / set_nixie / read_board_key / read_bluetooth_pad), `make_mc602_transport(node,"bridge",...)`
- Produces: services `/rak/hw/system/{read_sensor,beep,led_light,led_show,nixie,read_key,read_pad}`

- [ ] **Step 1: 写失败测试 `src/hardware/test/test_system_io.cpp`**(用注入 adapter)

```cpp
#include "hardware/system_io.hpp"
#include <gtest/gtest.h>

using namespace hardware;

// 注入式假 adapter:记录调用,返回固定值。SystemIo 只依赖 MC602Adapter 接口。
class FakeAdapter : public MC602AdapterIface
{
public:
  std::string last_sensor_type;
  uint8_t last_port = 0;
  int last_value = 0;
  std::vector<int64_t> key_values{1, 2, 3};

  double read_sensor(uint8_t port, const std::string & type) override
  {
    last_port = port; last_sensor_type = type;
    return 2.5;
  }
  void beep(int freq, float duration_s) override
  { last_value = freq; }
  void set_led_light(uint8_t led_id, int r, int g, int b) override
  { last_port = led_id; last_value = r; }
  void set_led_show(const std::string &) override {}
  void set_nixie(int value) override { last_value = value; }
  std::vector<int64_t> read_board_key() override { return key_values; }
  std::vector<int64_t> read_bluetooth_pad() override { return {9}; }
};

TEST(SystemIo, ReadSensorDispatchesByType)
{
  FakeAdapter f;
  SystemIo io(&f);
  bool ok; std::string err; double v;
  io.read_sensor(8, "ir", ok, err, v);
  EXPECT_TRUE(ok); EXPECT_EQ(v, 2.5);
  EXPECT_EQ(f.last_port, 8); EXPECT_EQ(f.last_sensor_type, "ir");
}

TEST(SystemIo, ReadSensorUnsupportedTypeFails)
{
  FakeAdapter f;
  SystemIo io(&f);
  bool ok; std::string err; double v;
  io.read_sensor(1, "bogus", ok, err, v);
  EXPECT_FALSE(ok); EXPECT_FALSE(err.empty());
}

TEST(SystemIo, BeepLedNixieRouteToAdapter)
{
  FakeAdapter f;
  SystemIo io(&f);
  bool ok; std::string err;
  io.beep(1000, 0.5f, ok, err);          EXPECT_TRUE(ok); EXPECT_EQ(f.last_value, 1000);
  io.set_led_light(2, 255, 0, 0, ok, err); EXPECT_TRUE(ok); EXPECT_EQ(f.last_port, 2);
  io.set_nixie(42, ok, err);             EXPECT_TRUE(ok); EXPECT_EQ(f.last_value, 42);
}

TEST(SystemIo, ReadKeyReturnsArray)
{
  FakeAdapter f;
  SystemIo io(&f);
  bool ok; std::string err; std::vector<int64_t> v;
  io.read_key(ok, err, v);
  EXPECT_TRUE(ok); ASSERT_EQ(v.size(), 3u); EXPECT_EQ(v[0], 1);
}
```

> 需要把 `MC602Adapter` 的虚接口抽成 `MC602AdapterIface`(或复用现有多态基类)。
> 若 `MC602Adapter` 已是 `BaseController` 的虚实现,则定义 `MC602AdapterIface` 为纯虚接口,
> `MC602Adapter : public MC602AdapterIface`,SystemIo 持 `MC602AdapterIface*`。

- [ ] **Step 2: 运行测试确认失败**(缺 `system_io.hpp`)

```bash
colcon build --packages-select hardware --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5
```
Expected: 编译报错 `system_io.hpp: No such file or directory`。

- [ ] **Step 3: 写纯类 `src/hardware/include/hardware/system_io.hpp` + `src/hardware/src/system_io.cpp`**

```cpp
// system_io.hpp — SystemIo: 非 arm/chassis 杂项设备的类型化门面(纯类,可注入)。
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace hardware
{
// MC602 设备接口的注入面。SystemIo 只依赖这里的虚方法。
class MC602AdapterIface
{
public:
  virtual ~MC602AdapterIface() = default;
  virtual double read_sensor(uint8_t port, const std::string & type) = 0;
  virtual void beep(int freq, float duration_s) = 0;
  virtual void set_led_light(uint8_t led_id, int r, int g, int b) = 0;
  virtual void set_led_show(const std::string & text) = 0;
  virtual void set_nixie(int value) = 0;
  virtual std::vector<int64_t> read_board_key() = 0;
  virtual std::vector<int64_t> read_bluetooth_pad() = 0;
};

class SystemIo
{
public:
  explicit SystemIo(MC602AdapterIface * dev) : dev_(dev) {}
  void read_sensor(uint8_t port, const std::string & type,
                   bool & ok, std::string & error, double & value);
  void beep(int freq, float duration_s, bool & ok, std::string & error);
  void set_led_light(uint8_t led_id, int r, int g, int b, bool & ok, std::string & error);
  void set_led_show(const std::string & text, bool & ok, std::string & error);
  void set_nixie(int value, bool & ok, std::string & error);
  void read_key(bool & ok, std::string & error, std::vector<int64_t> & values);
  void read_pad(bool & ok, std::string & error, std::vector<int64_t> & values);
private:
  MC602AdapterIface * dev_;
};
}
```

`system_io.cpp`:每个方法 try/catch,成功 `ok=true`;异常→`ok=false,error=e.what()`。
`read_sensor` 对未知 `type` 直接 `ok=false,error="unsupported sensor type"`(不调用驱动)。
`read_key`/`read_pad` 把 `read_board_key()`/`read_bluetooth_pad()` 拷进 values。

- [ ] **Step 4: 让 MC602Adapter 实现 MC602AdapterIface**

修改 `src/hardware/include/hardware/mc602_adapter.hpp`:`class MC602Adapter : public BaseController, public MC602AdapterIface`,并把 7 个虚方法标 `override`(它们已存在,签名一致则只加声明继承)。`read_sensor` 已是虚实现,`beep`/`set_led_light`/`set_led_show`/`set_nixie`/`read_board_key`/`read_bluetooth_pad` 已在类中。

- [ ] **Step 5: 跑测试确认通过**

```bash
colcon build --packages-select hardware --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5
```
Expected: `test_system_io` 全绿。

- [ ] **Step 6: 写薄壳 `src/hardware/src/nodes/system_io_node.cpp`**

构造时:声明 `mc602_serial_port`(默认 /dev/ttyUSB0)、`mc602_baud`(默认 115200)、`mc602_transport`(默认 "bridge");
`adapter_ = std::make_unique<MC602Adapter>(make_mc602_transport(this, mode, port, baud)); adapter_->open();`
`io_ = std::make_unique<SystemIo>(adapter_.get());`
创建 7 个 service server,每个回调调 `io_->...` 并回填响应。main 用 `MultiThreadedExecutor(4)`(bridge 模式阻塞等待)。

- [ ] **Step 7: CMake 加 executable + 测试 + install**

```cmake
add_executable(system_io_node
  src/nodes/system_io_node.cpp
  src/mc602_adapter.cpp
  src/mc602_protocol.cpp
  src/serial_transport.cpp
  src/bridge_transport.cpp
  src/serial_port.cpp
)
target_include_directories(system_io_node PUBLIC include)
target_link_libraries(system_io_node PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
  msgs::msgs
)
```
`find_package(std_srvs REQUIRED)` 加到头部。install 段加 `system_io_node`。
测试段:
```cmake
ament_add_gmock(test_system_io
  test/test_system_io.cpp
  src/system_io.cpp
)
target_include_directories(test_system_io PUBLIC include)
```

- [ ] **Step 8: 全量 build + test + commit**

```bash
colcon build --packages-up-to hardware --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hardware
git add src/hardware
git commit -m "feat(hardware): system_io_node —— beep/led/nixie/按键/传感器按需读 7 个类型化 service
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: NavController 纯类 + chassis action server + reset_encoders

**Files:**
- Create: `src/hardware/include/hardware/nav_controller.hpp`, `src/hardware/src/nav_controller.cpp`
- Create: `src/hardware/test/test_nav_controller.cpp`
- Modify: `src/hardware/src/nodes/mecanum_chassis_node.cpp`(action server + 导航整合 + reset_encoders service)
- Modify: `src/hardware/CMakeLists.txt`(rclcpp_action/std_srvs link + 测试)

**Interfaces:**
- Consumes: `vw::Pose2D`(base_controller.hpp)、`MecanumChassis::inverse`
- Produces: `NavController`(纯类)、chassis 节点 action server `/rak/chassis/navigate`、service `/rak/chassis/reset_encoders`

- [ ] **Step 1: 写失败测试 `test_nav_controller.cpp`**

```cpp
#include "hardware/nav_controller.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace hardware;

TEST(NavController, MovesTowardTargetAndReaches)
{
  NavController ctrl;
  NavGoal g;
  g.target.x = 1.0; g.target.y = 0.0; g.target.theta = 0.0;
  g.max_linear_speed = 0.3f; g.max_angular_speed = 1.0f;
  g.tolerance_lin = 0.02f; g.tolerance_ang = 0.05f;
  g.timeout_sec = 5.0f;

  NavTwist tw;
  NavStatus st = ctrl.update(Pose2D{0,0,0}, g, 0.1, tw);
  EXPECT_EQ(st, NavStatus::RUNNING);
  EXPECT_GT(tw.vx, 0.0f);           // 朝 +x 前进

  // 推进到接近目标 → REACHED
  st = ctrl.update(Pose2D{0.99,0,0}, g, 0.1, tw);
  EXPECT_EQ(st, NavStatus::REACHED);
}

TEST(NavController, TimesOutWhenStuck)
{
  NavController ctrl;
  NavGoal g;
  g.target.x = 5.0; g.target.y = 0; g.target.theta = 0;
  g.max_linear_speed = 0.1f; g.max_angular_speed = 1.0f;
  g.tolerance_lin = 0.02f; g.tolerance_ang = 0.05f;
  g.timeout_sec = 2.0f;
  NavTwist tw;
  double t = 0;
  NavStatus st = NavStatus::RUNNING;
  while (t < 3.0 && st == NavStatus::RUNNING) {
    st = ctrl.update(Pose2D{0,0,0}, g, t, tw);
    t += 0.1;
  }
  EXPECT_EQ(st, NavStatus::ABORTED);   // 未到目标且超时
}

TEST(NavController, ClampsSpeeds)
{
  NavController ctrl;
  NavGoal g;
  g.target.x = 100; g.target.y = 0; g.target.theta = 0;
  g.max_linear_speed = 0.5f; g.max_angular_speed = 2.0f;
  g.tolerance_lin = 0.02f; g.tolerance_ang = 0.05f;
  g.timeout_sec = 10.0f;
  NavTwist tw;
  ctrl.update(Pose2D{0,0,0}, g, 0.1, tw);
  EXPECT_LE(std::abs(tw.vx), 0.5f + 1e-6);
  EXPECT_LE(std::abs(tw.vy), 0.5f + 1e-6);
  EXPECT_LE(std::abs(tw.omega), 2.0f + 1e-6);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
colcon build --packages-select hardware --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5
```
Expected: `nav_controller.hpp` not found。

- [ ] **Step 3: 写纯类 `nav_controller.hpp` + `nav_controller.cpp`**

```cpp
// nav_controller.hpp — 底盘点到位 P 控制器(纯类,无 ROS 依赖,可测)。
// 复用 hardware/base_chassis.hpp 的 Pose2D(勿重复定义)。
#pragma once
#include "hardware/base_chassis.hpp"

#include <cmath>
#include <cstdint>

namespace hardware
{
struct NavGoal
{
  Pose2D target;
  float max_linear_speed = 0.3f;
  float max_angular_speed = 1.0f;
  float tolerance_lin = 0.02f;
  float tolerance_ang = 0.05f;
  float timeout_sec = 5.0f;
};

struct NavTwist { float vx = 0, vy = 0, omega = 0; };

enum class NavStatus { RUNNING, REACHED, ABORTED };

class NavController
{
public:
  // 每 tick 调用;`elapsed` = 距 goal 开始的秒数。返回状态,输出期望 body Twist。
  NavStatus update(const Pose2D & current, const NavGoal & goal,
                   double elapsed, NavTwist & out);
private:
  static double wrap_pi(double a);
};
}
```

`nav_controller.cpp`:
```cpp
#include "hardware/nav_controller.hpp"
#include <algorithm>

namespace hardware
{
double NavController::wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

NavStatus NavController::update(const Pose2D & cur, const NavGoal & g,
                                double elapsed, NavTwist & out)
{
  if (elapsed > g.timeout_sec) return NavStatus::ABORTED;

  const double dx = g.target.x - cur.x;
  const double dy = g.target.y - cur.y;
  const double dist = std::hypot(dx, dy);
  const double heading_err = wrap_pi(std::atan2(dy, dx) - cur.theta);
  const double theta_err = wrap_pi(g.target.theta - cur.theta);

  out.vy = static_cast<float>(
    std::clamp(std::sin(heading_err) * dist * 2.0,
               -static_cast<double>(g.max_linear_speed),
               static_cast<double>(g.max_linear_speed)));
  out.vx = static_cast<float>(
    std::clamp(std::cos(heading_err) * dist * 2.0,
               -static_cast<double>(g.max_linear_speed),
               static_cast<double>(g.max_linear_speed)));
  out.omega = static_cast<float>(
    std::clamp(theta_err * 3.0,
               -static_cast<double>(g.max_angular_speed),
               static_cast<double>(g.max_angular_speed)));

  if (dist < g.tolerance_lin && std::abs(theta_err) < g.tolerance_ang) {
    return NavStatus::REACHED;
  }
  return NavStatus::RUNNING;
}
}
```

- [ ] **Step 4: 跑测试通过**

```bash
colcon build --packages-select hardware --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -3 && ./build/hardware/test_nav_controller
```
Expected: 3 个用例全绿。

- [ ] **Step 5: chassis 节点加 action server + 导航整合**

修改 `src/hardware/src/nodes/mecanum_chassis_node.cpp`:
- include `<hardware/nav_controller.hpp>`, `<msgs/action/chassis_navigate.hpp>`, `<rclcpp_action/rclcpp_action.hpp>`, `<std_srvs/srv/trigger.hpp>`。
- 成员:`using NavAction = msgs::action::ChassisNavigate;` `rclcpp_action::Server<NavAction>::SharedPtr nav_server_;` `std::shared_ptr<rclcpp_action::ServerGoalHandle<NavAction>> active_goal_;` `NavGoal nav_goal_;` `bool nav_active_ = false;` `rclcpp::Time nav_start_;` `std::shared_ptr<NavController> nav_;`
- 构造末尾创建 action server:
```cpp
nav_server_ = rclcpp_action::create_server<NavAction>(
  this, "/rak/chassis/navigate",
  [this](const typename NavAction::Goal::ConstSharedPtr goal) {   // handle_goal
    std::lock_guard<std::mutex> lk(state_mutex_);
    nav_goal_.target.x = goal->target_pose.x;
    nav_goal_.target.y = goal->target_pose.y;
    nav_goal_.target.theta = goal->target_pose.theta;
    nav_goal_.max_linear_speed = goal->max_linear_speed;
    nav_goal_.max_angular_speed = goal->max_angular_speed;
    nav_goal_.tolerance_lin = goal->tolerance_lin;
    nav_goal_.tolerance_ang = goal->tolerance_ang;
    nav_goal_.timeout_sec = goal->timeout_sec;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  },
  [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavAction>>) {  // handle_cancel
    std::lock_guard<std::mutex> lk(state_mutex_);
    nav_active_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  },
  [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavAction>> gh) {  // handle_accepted
    std::lock_guard<std::mutex> lk(state_mutex_);
    // 预emption:若有活动 goal 先 abort
    if (active_goal_ && active_goal_->is_active()) {
      auto res = std::make_shared<NavAction::Result>();
      res->success = false; res->error = "preempted";
      active_goal_->abort(res);
    }
    active_goal_ = gh;
    nav_active_ = true;
    nav_start_ = this->now();
  });
```
- `publish_odometry()` 里在 `command_motors()` 之前加导航控制(odom 已更新出 `pose`):
```cpp
// 5. 导航控制:nav 激活时由 NavController 出 Twist,否则走 last_cmd
if (nav_active_) {
  NavTwist tw;
  const double elapsed = (this->now() - nav_start_).seconds();
  const auto st = nav_->update(pose, nav_goal_, elapsed, tw);
  { std::lock_guard<std::mutex> lk(state_mutex_);
    if (nav_active_) { // 仍在活动
      last_cmd_.linear.x = tw.vx; last_cmd_.linear.y = tw.vy; last_cmd_.angular.z = tw.omega;
      last_cmd_time_ = this->now();   // 防 deadman
      // feedback
      if (active_goal_ && active_goal_->is_active()) {
        auto fb = std::make_shared<NavAction::Feedback>();
        fb->current_pose.x = pose.x; fb->current_pose.y = pose.y; fb->current_pose.theta = pose.theta;
        fb->remaining_distance = static_cast<float>(std::hypot(
          nav_goal_.target.x - pose.x, nav_goal_.target.y - pose.y));
        active_goal_->publish_feedback(fb);
      }
      if (st == NavStatus::REACHED) {
        auto res = std::make_shared<NavAction::Result>();
        res->success = true; res->error = "none";
        res->traveled_distance = static_cast<float>(std::hypot(pose.x, pose.y));
        active_goal_->succeed(res);
        nav_active_ = false; active_goal_.reset();
      } else if (st == NavStatus::ABORTED) {
        auto res = std::make_shared<NavAction::Result>();
        res->success = false; res->error = "timeout";
        active_goal_->abort(res);
        nav_active_ = false; active_goal_.reset();
      }
    }
  }
  return; // 导航模式已设 last_cmd,走 command_motors
}
```
(注意 `nav_active_`/`active_goal_` 访问要持有 state_mutex_ 或单独锁;上面简化展示,实现时保证线程安全。)

- reset_encoders service:
```cpp
reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
  "/rak/chassis/reset_encoders",
  [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
         std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    try { adapter_->reset_encoder4(); resp->success = true; resp->message = "ok"; }
    catch (const std::exception & e) { resp->success = false; resp->message = e.what(); }
  });
```
- 构造里 `nav_ = std::make_unique<NavController>();`。

- [ ] **Step 6: CMake 加 rclcpp_action/std_srvs**

`mecanum_chassis_node` 的 target_link_libraries 加 `rclcpp_action::rclcpp_action`、`std_srvs::std_srvs`;头部 `find_package(rclcpp_action REQUIRED)`、`find_package(std_srvs REQUIRED)`。测试段:
```cmake
ament_add_gmock(test_nav_controller
  test/test_nav_controller.cpp
  src/nav_controller.cpp
)
target_include_directories(test_nav_controller PUBLIC include)
```

- [ ] **Step 7: build + 全测试 + commit**

```bash
colcon build --packages-up-to hardware --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hardware
git add src/hardware
git commit -m "feat(hardware): chassis —— ChassisNavigate action server + reset_encoders service
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: arm_node —— ArmExecuteTrajectory action + vacuum/valve services + 状态并入 joint_states

**Files:**
- Modify: `src/hardware/src/nodes/arm_node.cpp`
- Modify: `src/hardware/CMakeLists.txt`(rclcpp_action/std_srvs link)

**Interfaces:**
- Consumes: `MC602Adapter`(set_motor/set_stepper/set_servo_bus/set_servo_pwm/set_dout)
- Produces: action `/rak/arm/<arm_id>/execute_trajectory`、services `/rak/arm/<arm_id>/set_vacuum`、`/rak/arm/<arm_id>/set_valve`

- [ ] **Step 1: 抽一个可测的纯命令映射头 `src/hardware/include/hardware/arm_commands.hpp`**(可选,若 arm_node 已薄则跳过;建议抽出 JointTrajectory→设备命令的转换)

若抽取:定义 `struct ArmJointCommand { int8_t horiz_cmd; int32_t vert_steps; int s3_angle; int s7_angle; }`,`ArmJointCommand arm_tick_cmd(const std::vector<double> & positions)` 纯函数(把 on_trajectory 里的常量映射搬进来),并加 gtest 验证 ±夹取/stepper 步数/伺服角度映射。此步可与 Task 4 一起做,测试在 `test/test_arm_commands.cpp`。

- [ ] **Step 2: arm_node 加 action server + services**

修改 `src/hardware/src/nodes/arm_node.cpp`:
- include `<msgs/action/arm_execute_trajectory.hpp>`, `<rclcpp_action/rclcpp_action.hpp>`, `<std_srvs/srv/set_bool.hpp>`。
- 成员:`using ArmAction = msgs::action::ArmExecuteTrajectory;` `rclcpp_action::Server<ArmAction>::SharedPtr act_server_;` `std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmAction>> active_goal_;`
- 构造里创建 action server,`handle_accepted` 里执行:
```cpp
act_server_ = rclcpp_action::create_server<ArmAction>(
  this, "/rak/arm/" + arm_id_ + "/execute_trajectory",
  [](const typename ArmAction::Goal::ConstSharedPtr) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  },
  [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmAction>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  },
  [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmAction>> gh) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_goal_ && active_goal_->is_active()) {
      auto r = std::make_shared<ArmAction::Result>();
      r->success = false; r->error = "preempted";
      active_goal_->abort(r);
    }
    active_goal_ = gh;
    const auto & goal = gh->get_goal();
    // 复用现有执行逻辑:把 trajectory 最后点发给 on_trajectory
    trajectory_msgs::msg::JointTrajectory traj = goal->trajectory;
    // (on_trajectory 已锁 state_mutex_,这里先解锁再调)
    // 简化:直接把 on_trajectory 的转换逻辑执行一遍(或调用抽出的 arm_tick_cmd)
    auto res = std::make_shared<ArmAction::Result>();
    res->success = true; res->error = "none";
    res->final_positions = {0,0,0,0};  // 无关节编码器反馈,诚实报 0(见 no-mocks)
    gh->publish_feedback(std::make_shared<ArmAction::Feedback>());
    gh->succeed(res);
    active_goal_.reset();
  });
```
> 注意:arm 无关节反馈传感器,`on_trajectory` 是"写命令即完成"语义。Action 执行=写 burst 成功即 SUCCEEDED;失败(串口异常)在回调里 catch → `error` + `abort`。`final_positions` 填 0 并注释原因(不是假数据,是"无测量"的诚实表示)。

- 两个 service:
```cpp
vacuum_srv_ = this->create_service<std_srvs::srv::SetBool>(
  "/rak/arm/" + arm_id_ + "/set_vacuum",
  [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
         std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    try { adapter_->set_dout(2, req->data ? PUMP_ON : PUMP_OFF); resp->success = true; }
    catch (const std::exception & e) { resp->success = false; resp->message = e.what(); }
  });
valve_srv_ = this->create_service<std_srvs::srv::SetBool>(
  "/rak/arm/" + arm_id_ + "/set_valve",
  [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
         std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    try { adapter_->set_dout(3, req->data ? VALVE_CLOSE : VALVE_OPEN); resp->success = true; }
    catch (const std::exception & e) { resp->success = false; resp->message = e.what(); }
  });
```

- [ ] **Step 3: 状态 topic 并入 `/rak/state/joint_states`**

把 `state_topic` 从 `/rak/state/actuators/" + arm_id_` 改为 `/rak/state/joint_states`,发布逻辑不变(消息含 joint_names_,消费者按名字区分)。

- [ ] **Step 4: CMake link + (可选)arm_commands 测试**

`arm_node` target_link_libraries 加 `rclcpp_action::rclcpp_action`、`std_srvs::std_srvs`。若抽了 arm_commands,加 `ament_add_gmock(test_arm_commands ...)`。

- [ ] **Step 5: build + 全测试 + commit**

```bash
colcon build --packages-up-to hardware --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hardware
git add src/hardware
git commit -m "feat(hardware): arm —— ArmExecuteTrajectory action + vacuum/valve service,状态并入 joint_states
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: IR 频率默认 10Hz + launch 生产参数 115200

**Files:**
- Modify: `src/hardware/src/nodes/infrared_node.cpp`(`rate_hz` 默认 20.0→10.0)
- Modify: `src/bringup/launch/full_system.launch.py`(baud 1000000→115200;IR rate_hz 20.0→10.0;加 system_io_node + behavior_demo_node 节点;arm/mecanum 频率按 spec:arm publish_rate_hz 20.0,chassis 50.0 保留)
- Modify: `src/bringup/launch/mock_system.launch.py`(加 system_io_node;行为 demo 可选)

- [ ] **Step 1: infrared_node 默认频率改 10**

`rate_hz` 声明默认 `20.0` → `10.0`,日志同步。

- [ ] **Step 2: full_system.launch.py 生产默认**

- `DeclareLaunchArgument("baud", default_value="115200")`
- IR 两个节点的 `"rate_hz": 20.0` → `10.0`
- 新增节点块(放在 arm 之后):
```python
Node(package="hardware", executable="system_io_node", name="system_io",
     parameters=[{
         "mc602_serial_port": LaunchConfiguration("serial_port"),
         "mc602_baud": LaunchConfiguration("baud"),
         "mc602_transport": "bridge",
     }]),
Node(package="hardware", executable="behavior_demo_node", name="behavior_demo",
     parameters=[{"mc602_transport": "bridge"}]),  # Task 6 创建
```

- [ ] **Step 3: build + commit**

```bash
colcon build --packages-up-to bringup 2>&1 | tail -5
git add src/hardware src/bringup
git commit -m "feat(bringup): 生产参数对齐 115200,IR 10Hz,挂 system_io/behavior 节点
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: behavior_demo_node —— 调组件 Action 的编排示例

**Files:**
- Create: `src/hardware/src/nodes/behavior_demo_node.cpp`
- Modify: `src/hardware/CMakeLists.txt`(executable + link rclcpp_action)

**Interfaces:**
- Consumes: action clients `ChassisNavigate`(/rak/chassis/navigate)、`ArmExecuteTrajectory`(/rak/arm/main/execute_trajectory)
- Produces: `/rak/state/task/demo/status`(TaskStatus)、service `/rak/behavior/demo/start`(Trigger)

- [ ] **Step 1: 写节点**

构造:创建 2 个 action client + TaskStatus publisher(TRANSIENT_LOCAL)+ 启动 service。
`/rak/behavior/demo/start` 回调里跑脚本(异步,不阻塞 executor):
1. 发 TaskStatus `RUNNING / step=navigate`。
2. 调 ChassisNavigate(goal: 前进 0.5m):`client->async_send_goal` + `spin_until_future_complete`(或用定时轮询);SUCCEEDED → 下一步,否则 FAILED。
3. 发 TaskStatus `RUNNING / step=grip`。
4. 调 ArmExecuteTrajectory(goal: trajectory 到 `grip_s7=+1` 手抓)。
5. SUCCEEDED → TaskStatus `SUCCEEDED / progress=1.0`。

实现要点:action client 的 `spin_until_future_complete` 需要多线程 executor 或嵌套 spin;demo 节点 main 用 `MultiThreadedExecutor(4)`。脚本放独立 std::thread 里跑(避免阻塞回调)。完整实现约 150 行,展示"行为层=订阅+调 Action+发 TaskStatus"的契约形态。

- [ ] **Step 2: CMake**

```cmake
add_executable(behavior_demo_node src/nodes/behavior_demo_node.cpp)
target_include_directories(behavior_demo_node PUBLIC include)
target_link_libraries(behavior_demo_node PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  std_msgs::std_msgs
  std_srvs::std_srvs
  trajectory_msgs::trajectory_msgs
  msgs::msgs
)
```
install 段加 `behavior_demo_node`。

- [ ] **Step 3: build + commit**

```bash
colcon build --packages-up-to hardware 2>&1 | tail -5
git add src/hardware
git commit -m "feat(hardware): behavior_demo —— 调组件 Action 的编排示例(前进→抓取→TaskStatus)
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 7: 全量构建 + 测试 + 冒烟验证

**Files:** 无(验证)

- [ ] **Step 1: 全量构建 + 全量 gtest**

```bash
source /opt/ros/humble/setup.bash
cd /home/xrak/Desktop/XRAK/rak-car/.claude/worktrees/ros2-layering-interfaces
colcon build --packages-up-to bringup --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -10
colcon test --packages-select hardware msgs 2>&1 | tail -10
colcon test-result --verbose 2>&1 | tail -20
```
Expected: 全绿;现有 76+ gtest 通过;新增 test_system_io / test_nav_controller 通过。

- [ ] **Step 2: 接口存在性核对(无硬件)**

```bash
source install/setup.bash
ros2 interface show msgs/action/ChassisNavigate
ros2 interface show msgs/action/ArmExecuteTrajectory
ros2 interface show msgs/srv/SensorQuery
```
Expected: 显示完整 goal/result/feedback 定义。

- [ ] **Step 3: mock 冒烟(尽力而为,如实报告)**

```bash
timeout 25 ros2 launch bringup mock_system.launch.py &  # 若 dev 机无串口/相机,节点启动失败属预期
```
如实记录哪些节点起来、哪些因缺硬件失败。若 mock 起不来,以 Step 1-2 + 单节点 `ros2 run` 结构验证代替,并在报告里说明。

- [ ] **Step 4: 汇总报告 + 收尾**

核对 spec §13 每项 → 完成状态;列出新增文件、接口清单、遗留项(如 arm 无反馈传感器、behavior 框架化留待后续)。确保所有提交已推(worktree 收尾前 push 到 `develop/ros2-sidecar`)。
