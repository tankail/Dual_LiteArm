# LiteArm A10 双臂 ROS2 控制

## 目录结构

```
.
├── src/
│   ├── litearm_config/           # MoveIt 配置与启动文件
│   │   ├── config/               # URDF/Xacro、SRDF、控制器 YAML、RViz 配置
│   │   ├── launch/               # 所有启动文件
│   │   ├── scripts/              # Python 工具脚本
│   │   └── robot_param/          # 机械臂参数与电机配置
│   ├── litearm_a10_251125/       # 机械臂 URDF 描述（含 mesh）
│   ├── litearm_hardware/         # ros2_control 硬件接口插件
│   └── litearm_robot/            # 电机控制 SDK + 工具程序
│       └── examples/             # SDK 示例（角度读取、重力补偿等）
├── build/
└── install/
```

### 关键文件速查

| 文件 | 用途 |
|------|------|
| `launch/hardware_moveit_rviz.launch.py` | **右臂** MoveIt + RViz |
| `launch/hardware_moveit_rviz_left_arm.launch.py` | **左臂** MoveIt + RViz |
| `launch/hardware_moveit_rviz_dual.launch.py` | **双臂** MoveIt + RViz（position_velocity 模式） |
| `launch/hardware_moveit_rviz_dual_gravity.launch.py` | **双臂** 重力补偿模式（full_control + Pinocchio） |
| `launch/hardware.launch.py` | 右臂底层硬件（不含 MoveIt） |
| `launch/hardware_left_arm.launch.py` | 左臂底层硬件（不含 MoveIt） |
| `launch/hardware_dual.launch.py` | 双臂底层硬件（不含 MoveIt） |
| `scripts/joint_angle_monitor.py` | 终端实时显示关节角度（度）— Python，订阅 /joint_states |
| `examples/dual_arm_joint_angle.cpp` | 终端实时显示关节角度（度）— C++，SDK 直读 |
| `examples/dual_arm_aging_test.cpp` | 双臂老化循环测试 |
| `examples/left_arm_gravity_compensation.cpp` | 左臂纯重力补偿（Pinocchio RNEA） |
| `examples/right_arm_gravity_compensation.cpp` | 右臂纯重力补偿（Pinocchio RNEA） |
| `config/ros2_controllers_hardware_dual.yaml` | 双臂 ros2_control 控制器配置 |
| `config/moveit_controllers_hardware_dual.yaml` | 双臂 MoveIt 控制器映射 |
| `config/LiteArm_A10_251125.srdf` | MoveIt 语义描述（Planning Group、命名姿态） |
| `robot_param/litearm_left_arm_motors.yaml` | 左臂电机型号与串口配置 |
| `robot_param/litearm_right_arm_motors.yaml` | 右臂电机型号与串口配置 |

---

## 电机配置

### 电机型号映射

| 关节 | 左臂 | 右臂 | 电机型号 | max_torque | kp | kd |
|------|------|------|----------|:---:|:--:|:--:|
| joint1 | l_joint1 | r_joint1 | 7256_35 | 72 Nm | 300 | 10 |
| joint2 | l_joint2 | r_joint2 | 7256_35 | 72 Nm | 300 | 10 |
| joint3 | l_joint3 | r_joint3 | 6056_36 | 48 Nm | 200 | 8 |
| joint4 | l_joint4 | r_joint4 | 6056_36 | 48 Nm | 200 | 8 |
| joint5 | l_joint5 | r_joint5 | 4438_30 | 20 Nm | 60 | 3 |
| joint6 | l_joint6 | r_joint6 | 5047_36 | 36 Nm | 100 | 5 |
| joint7 | l_joint7 | r_joint7 | 4438_30 | 20 Nm | 60 | 3 |
| gripper | l_r_finger | r_r_finger | 4438_30 | 10 Nm | 3 | 0.3 |

kp/kd 配置在 `config/LiteArm_A10_251125_hardware.ros2_control.xacro`（右臂）和 `config/LiteArm_A10_251125_hardware_left_arm.ros2_control.xacro`（左臂）。

---

## 串口配置

### 设备与 serial_id 映射

硬件插件通过 `robot_param/litearm_*_motors.yaml` 中的 `serial_id` 来确定使用哪个 `/dev/ttyACM` 设备（`serial_id` 为 1-based 索引）：

```yaml
robot:
  Serial_Type: "/dev/ttyACM"
  ...
  CANboard:
    No_1_CANboard:
      CANport:
        CANport_1:
          serial_id: <N>   # 索引到第 N 个 /dev/ttyACM 设备
```

| 机械臂 | serial_id | 对应端口 |
|--------|-----------|----------|
| 右臂   | 1         | /dev/ttyACM0 |
| 左臂   | 2         | /dev/ttyACM1 |

> **注意**：`serial_id` 值不能为 0；映射关系由 SDK 的 `robot.cpp:415`（`str[serial_id-1]`）决定。

### 验证串口连接

```bash
ls -la /dev/ttyACM*

# 权限检查
groups $USER
sudo usermod -a -G dialout $USER   # 如无权限，执行后重新登录
```

---

## 启动方式

### 编译

```bash
cd ~/Dual_LiteArm
colcon build --packages-select litearm_config litearm_a10_251125 litearm_hardware litearm_robot
source install/setup.bash
```

### 双臂重力补偿模式（推荐）

```bash
ros2 launch litearm_config hardware_moveit_rviz_dual_gravity.launch.py
```

- 使用 `full_control` 模式
- Pinocchio RNEA 实时计算重力力矩前馈
- 电机内部 PID（kHz 级）跟踪 MoveIt 轨迹
- 效果：手臂既跟轨迹又抵消自重

### 双臂标准模式

```bash
ros2 launch litearm_config hardware_moveit_rviz_dual.launch.py

# 不带 RViz
ros2 launch litearm_config hardware_moveit_rviz_dual.launch.py rviz:=false
```

### 单臂启动

```bash
# 右臂
ros2 launch litearm_config hardware_moveit_rviz.launch.py

# 左臂
ros2 launch litearm_config hardware_moveit_rviz_left_arm.launch.py
```

### 仅硬件（不含 MoveIt）

```bash
ros2 launch litearm_config hardware.launch.py           # 右臂
ros2 launch litearm_config hardware_left_arm.launch.py  # 左臂
ros2 launch litearm_config hardware_dual.launch.py      # 双臂
```

---

## 关节角度读取工具

### 方法一：C++ SDK 直读（不能和 ros2_control 同时用）

直接通过串口读取电机状态，显示角度（度）、速度、力矩：

```bash
ros2 run litearm_robot dual_arm_joint_angle            # 双臂
ros2 run litearm_robot dual_arm_joint_angle --left     # 仅左臂
ros2 run litearm_robot dual_arm_joint_angle --right    # 仅右臂
```

输出示例：
```
======================================================================
  LiteArm 双臂关节角度显示器 (SDK直接读取)
  Sample #23  |  刷新频率: 5.1 Hz
======================================================================

  [左臂]
  --------------------------------------------------------------------
  关节               角度(°)      速度(°/s)     力矩(Nm)    位置指示
  --------------------------------------------------------------------
  l_joint1_joint      -22.00          0.05         0.12   [       |  *       ]
  l_joint2_joint       -2.00          0.01        -0.08   [       |*         ]
  ...
```

### 方法二：Python 订阅 /joint_states（可和 ros2_control 同时用）

从 ROS2 话题读取，显示角度（度），带可视化条：

```bash
ros2 run litearm_config joint_angle_monitor.py          # 双臂
ros2 run litearm_config joint_angle_monitor.py --left   # 仅左臂
ros2 run litearm_config joint_angle_monitor.py --right  # 仅右臂
```

### 方法三：ROS2 CLI

```bash
ros2 topic echo /joint_states --field position
```

---

## 控制模式

| 模式 | 启动方式 | 驱动函数 | 说明 |
|------|----------|----------|------|
| `position_velocity` | 默认 | `posVelMaxTorque()` | 位置+速度，电机内部 PID，**不使用** xacro 的 kp/kd |
| `pd_control` | `control_mode:=pd_control` | `posVelTorqueKpKd()` | MIT 模式，发 kp/kd，无前馈力矩 |
| `full_control` | `control_mode:=full_control` | `posVelTorqueKpKd()` | MIT 模式，**发 kp/kd + 重力补偿力矩 G(q)** |

重力补偿 launch 文件默认使用 `full_control` 模式。

### 重力补偿工作原理

```
每个控制周期 (@100Hz):
  1. 读当前关节位置 q
  2. Pinocchio RNEA → 计算重力力矩 G(q)
  3. MoveIt → 目标位置 q_des、速度 v_des
  4. 发送: posVelTorqueKpKd(q_des, v_des, G(q), kp, kd)
  5. 电机固件 (kHz): torque = kp*(q_des-q) + kd*(v_des-v) + G(q)
```

重力增益参数在 xacro 的 `<param name="gravity_gain">` 中：左臂 0.85，右臂 1.0。可单独调。

---

## 命名姿态（SRDF）

在 RViz Motion Planning 面板的 Start/Goal State 下拉中可直接选择：

| 姿态名 | Planning Group | 关节值（度） |
|--------|:---:|------|
| `home` | left_arm | (0, 0, 0, 0, 0, 0, 0) |
| `pose_A` | left_arm | (-22, -2, 12, -66, 2, -2, -16) |
| `pose1` | left_arm | (62, -15, 10, -114, 7, -10, -36) |
| `pose2` | left_arm | (-38, 1, 15, -70, -3, -2, -18) |
| `pose3` | left_arm | (-30, 8, 15, -94, 10, -2.5, 30) |
| `pose4` | left_arm | (-90, 0, 0, 0, 0, 0, 0) |

姿态定义在 `config/LiteArm_A10_251125.srdf` 中，单位为弧度。

---

## 控制器与 Planning Group

### ros2_control 控制器

| 控制器名 | 类型 | 控制的关节 |
|----------|------|-----------|
| `joint_state_broadcaster` | JointStateBroadcaster | 所有关节状态 |
| `left_arm_controller` | JointTrajectoryController | `l_joint1~7_joint` |
| `left_gripper_controller` | JointTrajectoryController | `l_r_finger_joint` |
| `right_arm_controller` | JointTrajectoryController | `r_joint1~7_joint` |
| `right_gripper_controller` | JointTrajectoryController | `r_r_finger_joint` |

### MoveIt Planning Group

| Group | 关节 | End Effector |
|-------|------|:---:|
| `left_arm` | l_joint1~7 | `left_effector` (l_gripper_link) |
| `right_arm` | r_joint1~7 | `right_effector` (r_gripper_link) |
| `left_gripper` | l_r_finger, l_l_finger | - |
| `right_gripper` | r_r_finger, r_l_finger | - |

---

## 双臂架构

```
hardware_moveit_rviz_dual_gravity.launch.py
├── hardware_dual.launch.py
│   ├── robot_state_publisher          # TF 树（双臂）
│   ├── ros2_control_node              # 单个 controller_manager 管理双臂
│   │   ├── LiteArmLeftArmHardware     # 左臂硬件插件
│   │   │   ├── Pinocchio 模型（共享 URDF）
│   │   │   └── 重力力矩 G_l(q) = RNEA(q, offset=0)
│   │   └── LiteArmRightArmHardware    # 右臂硬件插件
│   │       ├── Pinocchio 模型（共享 URDF）
│   │       └── 重力力矩 G_r(q) = RNEA(q, offset=9)
│   └── spawners
│       ├── joint_state_broadcaster
│       ├── left_arm_controller  ←┐ 并行
│       ├── right_arm_controller ←┘
│       ├── left_gripper_controller
│       └── right_gripper_controller
├── static_virtual_joint_tfs           # world → base_link
├── move_group                         # MoveIt
└── rviz2                              # RViz
```

---

## 常见问题

### 1. controller_manager 崩溃 (std::bad_alloc)

```
[ERROR] Memory allocation failed: std::bad_alloc
```

**原因**：`serial_id` 为 0 或电机型号字符串不在 SDK 支持的列表中。
检查 `robot_param/litearm_*_motors.yaml` 中的 `serial_id` 和电机 `type`。

### 2. 端口权限不足

```bash
sudo usermod -a -G dialout $USER  # 重新登录后生效
```

### 3. MoveIt 无法获取关节状态

```
[WARN] Didn't receive robot state with recent timestamp
```

**原因**：`controller_manager` 崩溃 → `joint_state_broadcaster` 未运行。查看上方日志。

### 4. 电机太软 / 达不到目标位置

- 重力补偿模式：检查 `gravity_gain` 是否偏小
- kp 值受 CAN 协议 int16 限制（最大有效值 ~257-352），已设置为不超过上限
- 标准 `position_velocity` 模式：检查 `max_torque` 是否偏小

### 5. 手臂抖动

- kp 过大 → 在 xacro 中降低对应关节的 kp
- kd 过小 → 适当增大 kd（约 kp * 0.03 ~ 0.05）
