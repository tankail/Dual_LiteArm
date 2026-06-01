# LiteArm ROS2 工作空间

## 目录结构

```
src/
├── litearm_a10_251125/      # LiteArm A10 机械臂 URDF 和描述文件
├── litearm_config/          # MoveIt 配置文件、launch 文件、YAML 参数
├── litearm_examples/        # 示例代码
├── litearm_hardware/        # 硬件接口插件（ros2_control）
├── litearm_robot/           # 核心 SDK 和控制例程
└── litearm_robot/urdf/      # 右臂 URDF（用于重力补偿计算）
```

## 右手臂重力补偿

### 原理

通过 Pinocchio 计算当前姿态下的重力补偿力矩 G(q)，发送给电机实现零重力拖动效果：

```
τ = G(q)
```

其中：
- `q` - 当前关节角度（7个关节）
- `G(q)` - RNEA 算法计算的重力力矩

### 相关文件

| 文件 | 说明 |
|------|------|
| `litearm_robot/urdf/LiteArm_A10_251224_right_arm.urdf` | 右臂 URDF（含质量、惯量、关节轴方向） |
| `litearm_robot/examples/right_arm_gravity_calc.cpp` | 重力补偿计算器（只打印，不输出力矩） |
| `litearm_robot/examples/right_arm_gravity_compensation.cpp` | 重力补偿控制器（计算并输出力矩） |
| `litearm_config/robot_param/litearm_right_arm.yaml` | 右臂电机配置文件 |

### 运行方法

```bash
# 1. 进入工作空间
cd ~/LiteArm

# 2. 编译
colcon build --packages-select litearm_robot
source install/setup.bash

# 3. 仅计算重力补偿力矩（安全诊断）
ros2 run litearm_robot right_arm_gravity_calc

# 4. 启用重力补偿输出
ros2 run litearm_robot right_arm_gravity_compensation
```

### 参数调整

编辑 `right_arm_gravity_compensation.cpp` 中的 `gravity_gain` 参数：

```cpp
// 第 119 行
std::vector<double> gravity_gain = {1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0};
//                  [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
```

- `> 1.0` - 力矩放大
- `< 1.0` - 力矩缩小
- `= 1.0` - 不变

**调参步骤**：
1. 松手让手臂在某个姿态下自然悬停
2. 观察手臂往哪个方向漂移
3. 往上漂移 → 减小该关节增益；往下掉 → 增大该关节增益
4. 修改后重新编译

### URDF 关节轴方向说明

如果发现某个关节的力矩方向始终反的，需要修改 `LiteArm_A10_251224_right_arm.urdf` 中的 `axis` 属性：

| 关节 | 当前 axis | 说明 |
|------|-----------|------|
| joint1 | `0 1 0` | 绕 Y 轴旋转 |
| joint2 | `1 0 0` | 绕 X 轴旋转 |
| joint3 | `0 0 -1` | 绕 Z 轴负方向旋转 |
| joint4 | `0 1 0` | 绕 Y 轴旋转 |
| joint5 | `0 0 -1` | 绕 Z 轴负方向旋转 |
| joint6 | `1 0 0` | 绕 X 轴旋转 |
| joint7 | `0 1 0` | 绕 Y 轴旋转 |

修改后重新编译：`colcon build --packages-select litearm_robot`

### 力矩安全限幅

| 关节 | 电机型号 | 限幅 (Nm) | 当前增益 |
|------|----------|-----------|----------|
| joint1 | 5047_36 | 15.0 | 1.0 |
| joint2 | 6056_36 | 25.0 | 1.2 |
| joint3 | 6056_36 | 25.0 | 1.0 |
| joint4 | 5047_36 | 15.0 | 0.8 |
| joint5 | 4438_30 | 6.0 | 1.0 |
| joint6 | 4438_30 | 6.0 | 1.0 |
| joint7 | 4438_30 | 4.0 | 1.0 |


## 左手臂重力补偿

### 原理

与右臂相同，通过 Pinocchio RNEA 算法计算当前姿态下的重力补偿力矩 G(q)。

### 相关文件

| 文件 | 说明 |
|------|------|
| litearm_robot/urdf/LiteArm_A10_251224_left_arm.urdf | 左臂 URDF（运动学/动力学参数） |
| litearm_robot/urdf/LiteArm_A10_251224_left_arm_display.urdf | 左臂显示 URDF（含 STL 网格） |
| litearm_robot/examples/left_arm_gravity_calc.cpp | 左臂重力补偿计算器（只打印） |
| litearm_robot/examples/left_arm_gravity_compensation.cpp | 左臂重力补偿控制器（输出力矩） |
| litearm_config/robot_param/litearm_left_arm.yaml | 左臂配置文件 |
| litearm_config/robot_param/litearm_left_arm_motors.yaml | 左臂电机参数（serial_id 等） |

### 运行方法

```bash
# 仅计算（安全诊断）
ros2 run litearm_robot left_arm_gravity_calc

# 启用重力补偿输出
ros2 run litearm_robot left_arm_gravity_compensation
```

### 力矩安全限幅

| 关节 | 电机型号 | 限幅 (Nm) | 当前增益 |
|------|----------|-----------|----------|
| joint1 (5047_36) | l_joint1 | 15.0 | 0.85 |
| joint2 (6056_36) | l_joint2 | 25.0 | 1.0 |
| joint3 (6056_36) | l_joint3 | 25.0 | 1.0 |
| joint4 (5047_36) | l_joint4 | 15.0 | 0.8 |
| joint5 (4438_30) | l_joint5 | 6.0 | 1.0 |
| joint6 (4438_30) | l_joint6 | 6.0 | 1.0 |
| joint7 (4438_30) | l_joint7 | 4.0 | 1.0 |

## 左手臂 MoveIt 控制

### 相关文件

| 文件 | 说明 |
|------|------|
| litearm_config/launch/hardware_moveit_rviz_left_arm.launch.py | 左臂 MoveIt 启动（含硬件、MoveGroup、RViz） |
| litearm_config/launch/hardware_left_arm.launch.py | 左臂硬件启动（不含 MoveIt） |
| litearm_config/config/LiteArm_A10_251125_hardware_left_arm.urdf.xacro | 左臂硬件 URDF xacro |
| litearm_config/config/LiteArm_A10_251125_hardware_left_arm.ros2_control.xacro | 左臂 ros2_control 配置 |
| litearm_config/config/ros2_controllers_hardware_left_arm.yaml | 左臂控制器参数 |
| litearm_config/config/moveit_controllers_hardware_left_arm.yaml | 左臂 MoveIt 控制器映射 |
| litearm_config/config/kinematics.yaml | 运动学求解器配置（含 left_arm） |
| litearm_config/config/LiteArm_A10_251125.srdf | 语义描述（含 left_arm 组） |

### 运行方法

```bash
# 启动左臂 MoveIt + RViz
ros2 launch litearm_config hardware_moveit_rviz_left_arm.launch.py

# 仅启动硬件（不含 MoveIt）
ros2 launch litearm_config hardware_left_arm.launch.py
```

### URDF 关节轴方向

修正后左臂关节轴与电机方向一致：

| 关节 | axis | 限位 (rad) |
|------|------|-----------|
| l_joint1 | 0 1 0 | [-3.05, 1.66] |
| l_joint2 | -1 0 0 | [-2.86, 0.28] |
| l_joint3 | 0 0 -1 | [-1.57, 1.57] |
| l_joint4 | 0 1 0 | [-2.44, 1.57] |
| l_joint5 | 0 0 -1 | [-1.57, 1.57] |
| l_joint6 | -1 0 0 | [-0.77, 0.77] |
| l_joint7 | 0 1 0 | [-1.57, 1.57] |

## 显示完整机器人模型

```bash
ros2 launch litearm_robot display_robot.launch.py
```

使用 `litearm_a10_251125/urdf/LiteArm_A10_251125.urdf`（双臂 URDF），包含左右臂各 7 个关节、夹爪和手指的 STL 网格模型。可通过 `joint_state_publisher_gui` 拖动滑块调节所有关节角度。

## 其他例程

```bash
# 读取电机状态
ros2 run litearm_robot 0_robot_get_state

# PD 控制
ros2 run litearm_robot 1_PD_control

# 位置速度控制
ros2 run litearm_robot 1_PosVel_control

# 关节空间阻抗控制（需 Pinocchio）
ros2 run litearm_robot 2_joint_impedance_control

# 笛卡尔空间阻抗控制（需 Pinocchio）
ros2 run litearm_robot 3_cartesian_impedance_control
```

## 注意事项

1. **退出时电机掉电** - 按 Ctrl+C 退出后，电机将停止输出力矩，请注意安全
2. **初始姿态** - 建议在运行重力补偿前将手臂摆放到一个安全的初始姿态
3. **观测调参** - 建议先用 `right_arm_gravity_calc` 观察不同姿态下的力矩值，再进行调参