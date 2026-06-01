#!/usr/bin/python3.10
"""
老化测试脚本 - 右臂循环运动测试
让右臂7个关节和夹爪在两个位置之间来回运动
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup
import time


class AgingTestNode(Node):
    def __init__(self):
        super().__init__('aging_test_node')

        # 回调组
        self.callback_group = ReentrantCallbackGroup()

        # 位置定义 (单位: 度)
        # 起始位置 - 全部归零
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_closed = 0.0

        # 目标位置
        self.target_position = [84.0, 70.0, -67.0, 105.0, 90.0, 37.0, 44.0]
        self.gripper_open = 0.038  # 夹爪张开

        # 创建 publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/right_gripper_controller/joint_trajectory',
            10
        )

        # 等待连接
        self.get_logger().info('等待右臂控制器发布者连接...')
        timeout = 5.0
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['r_joint1_joint']
            self.arm_pub.publish(msg)
            time.sleep(0.1)
            if self.arm_pub.get_subscription_count() > 0:
                break
        self.get_logger().info('右臂控制器已连接')

        self.get_logger().info('等待夹爪控制器发布者连接...')
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['r_r_finger_joint']
            self.gripper_pub.publish(msg)
            time.sleep(0.1)
            if self.gripper_pub.get_subscription_count() > 0:
                break
        self.get_logger().info('夹爪控制器已连接')

        # 统计
        self.cycle_count = 0
        self.start_time = time.time()

    def deg_to_rad(self, degrees):
        """度转弧度"""
        return [d * 3.14159265359 / 180.0 for d in degrees]

    def send_arm_trajectory(self, positions, duration=2.0):
        """发送手臂轨迹 - 直接发布到话题"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'r_joint1_joint', 'r_joint2_joint', 'r_joint3_joint',
            'r_joint4_joint', 'r_joint5_joint', 'r_joint6_joint',
            'r_joint7_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = self.deg_to_rad(positions)
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()

        msg.points.append(point)
        self.arm_pub.publish(msg)
        self.get_logger().info(f'手臂发布轨迹: {positions}')

    def send_gripper_trajectory(self, position, duration=1.0):
        """发送夹爪轨迹 - 直接发布到话题"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['r_r_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()

        msg.points.append(point)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'夹爪发布轨迹: {position:.4f}')

    def run_cycle(self):
        """执行一次循环"""
        self.cycle_count += 1
        self.get_logger().info(f'\n=== 循环 #{self.cycle_count} ===')

        # 1. 手臂移动到目标位置 (3秒)
        self.get_logger().info('1. 手臂移动到目标位置...')
        self.send_arm_trajectory(self.target_position, duration=3.0)
        time.sleep(3.5)

        # 2. 夹爪张开 (1秒)
        self.get_logger().info('2. 夹爪张开...')
        self.send_gripper_trajectory(self.gripper_open, duration=1.0)
        time.sleep(1.5)

        # 3. 夹爪关闭 (1秒)
        self.get_logger().info('3. 夹爪关闭...')
        self.send_gripper_trajectory(self.gripper_closed, duration=1.0)
        time.sleep(1.5)

        # 4. 手臂回到起始位置 (3秒)
        self.get_logger().info('4. 手臂回到起始位置...')
        self.send_arm_trajectory(self.home_position, duration=3.0)
        time.sleep(3.5)

        # 打印统计
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'完成循环 #{self.cycle_count}, '
            f'总运行时间: {elapsed:.1f}秒, '
            f'频率: {self.cycle_count/elapsed*3600:.1f}循环/小时'
        )

    def run(self, max_cycles=None):
        """运行老化测试"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('老化测试开始!')
        self.get_logger().info(f'目标位置: {self.target_position}')
        self.get_logger().info(f'起始位置: {self.home_position}')
        self.get_logger().info(f'夹爪张开: {self.gripper_open}, 关闭: {self.gripper_closed}')
        self.get_logger().info('=' * 50)

        try:
            while rclpy.ok():
                if max_cycles and self.cycle_count >= max_cycles:
                    break
                self.run_cycle()

        except KeyboardInterrupt:
            self.get_logger().info('测试被用户中断')

        finally:
            elapsed = time.time() - self.start_time
            self.get_logger().info('=' * 50)
            self.get_logger().info('老化测试结束')
            self.get_logger().info(f'总循环次数: {self.cycle_count}')
            self.get_logger().info(f'总运行时间: {elapsed:.1f}秒 ({elapsed/3600:.2f}小时)')
            if elapsed > 0:
                self.get_logger().info(f'平均频率: {self.cycle_count/elapsed*3600:.1f}循环/小时')
            self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)

    # 解析参数
    import sys
    max_cycles = None
    if len(sys.argv) > 1:
        try:
            max_cycles = int(sys.argv[1])
            print(f'将运行 {max_cycles} 个循环')
        except ValueError:
            print(f'无效参数: {sys.argv[1]}, 将无限循环')

    node = AgingTestNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # 在单独线程中运行 spin
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 主循环
        node.run(max_cycles=max_cycles)

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
