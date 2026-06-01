#!/usr/bin/env python3
"""
关节角度显示器 - 从 /joint_states 读取并显示双臂关节角度（度）

用法：
  ros2 run litearm_config joint_angle_monitor.py
  ros2 run litearm_config joint_angle_monitor.py --left    # 只显示左臂
  ros2 run litearm_config joint_angle_monitor.py --right   # 只显示右臂
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import os
import sys


class JointAngleMonitor(Node):
    def __init__(self, arm_filter=None):
        super().__init__('joint_angle_monitor')
        self.arm_filter = arm_filter  # None, 'left', 'right'

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.callback, 10)

        self.last_print = self.get_clock().now()
        self.print_interval = 0.5  # 0.5秒刷新一次

        self.get_logger().info('关节角度显示器已启动，等待 /joint_states ...')

    def rad_to_deg(self, rad):
        return rad * 180.0 / math.pi

    def callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds * 1e-9 < self.print_interval:
            return
        self.last_print = now

        # 分类关节
        left_arm = {}
        left_gripper = {}
        right_arm = {}
        right_gripper = {}

        for name, pos in zip(msg.name, msg.position):
            if name.startswith('l_joint') and name.endswith('_joint'):
                left_arm[name] = pos
            elif name in ('l_r_finger_joint', 'l_l_finger_joint'):
                left_gripper[name] = pos
            elif name.startswith('r_joint') and name.endswith('_joint'):
                right_arm[name] = pos
            elif name in ('r_r_finger_joint', 'r_l_finger_joint'):
                right_gripper[name] = pos

        os.system('clear')

        print("=" * 70)
        print("   LiteArm 关节角度显示器 (单位: 度)")
        print("=" * 70)

        # 左臂
        if self.arm_filter is None or self.arm_filter == 'left':
            print("\n  [左臂]")
            print("  " + "-" * 40)
            arm_joints = ['l_joint1_joint', 'l_joint2_joint', 'l_joint3_joint',
                          'l_joint4_joint', 'l_joint5_joint', 'l_joint6_joint',
                          'l_joint7_joint']
            for j in arm_joints:
                if j in left_arm:
                    deg = self.rad_to_deg(left_arm[j])
                    bar = self._make_bar(deg, -180, 180)
                    print(f"  {j:22s}: {deg:8.2f}° {bar}")
            if left_gripper:
                print("  " + "-" * 40)
                for name, pos in sorted(left_gripper.items()):
                    print(f"  {name:22s}: {pos*1000:8.3f} mm")

        # 右臂
        if self.arm_filter is None or self.arm_filter == 'right':
            print("\n  [右臂]")
            print("  " + "-" * 40)
            arm_joints = ['r_joint1_joint', 'r_joint2_joint', 'r_joint3_joint',
                          'r_joint4_joint', 'r_joint5_joint', 'r_joint6_joint',
                          'r_joint7_joint']
            for j in arm_joints:
                if j in right_arm:
                    deg = self.rad_to_deg(right_arm[j])
                    bar = self._make_bar(deg, -180, 180)
                    print(f"  {j:22s}: {deg:8.2f}° {bar}")
            if right_gripper:
                print("  " + "-" * 40)
                for name, pos in sorted(right_gripper.items()):
                    print(f"  {name:22s}: {pos*1000:8.3f} mm")

        print("\n" + "=" * 70)
        print("  按 Ctrl+C 退出")

    def _make_bar(self, value, vmin, vmax, width=15):
        """简易可视化条"""
        if value < vmin:
            value = vmin
        if value > vmax:
            value = vmax
        if vmax == vmin:
            return "|" + " " * width + "|"

        ratio = (value - vmin) / (vmax - vmin)
        pos = int(ratio * width)
        bar_chars = list(" " * width)
        bar_chars[pos] = "*"
        return "|" + "".join(bar_chars) + "|"


def main(args=None):
    rclpy.init(args=args)

    arm_filter = None
    if '--left' in sys.argv:
        arm_filter = 'left'
    elif '--right' in sys.argv:
        arm_filter = 'right'

    node = JointAngleMonitor(arm_filter=arm_filter)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
