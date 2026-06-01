/**
 * @file dual_arm_aging_test.cpp
 * @brief 双臂老化测试程序
 *
 * 让左右手臂同时在起始关节角度和终点关节角度之间来回运动，不停止
 *
 * 使用方法：
 *   ros2 run litearm_robot dual_arm_aging_test
 */

#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>

std::atomic<bool> keep_running(true);

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        keep_running = false;
        std::cout << "\n\n收到退出信号，正在停止..." << std::endl;
    }
}

// 角度转弧度
double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

// 打印当前关节角度
void printCurrentAngles(litearm_robot::LiteArm& robot, const std::string& arm_name, int n)
{
    robot.send_get_motor_state_cmd();
    robot.motor_send_cmd();
    auto pos = robot.getCurrentPos();

    std::cout << "[" << arm_name << "] 当前角度(°): [";
    for (int i = 0; i < n; ++i) {
        double angle_deg = pos[i] * 180.0 / M_PI;
        std::cout << std::fixed << std::setprecision(1) << angle_deg;
        if (i < n - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

int main(int argc, char** argv)
{
    try {
        signal(SIGINT, signal_handler);

        // 配置文件路径
        std::string right_config = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_right_arm.yaml";
        std::string left_config = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_left_arm.yaml";

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "双臂老化测试程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "右手臂配置: " << right_config << std::endl;
        std::cout << "左手臂配置: " << left_config << std::endl;

        // 初始化双臂
        std::cout << "\n初始化右手臂..." << std::endl;
        litearm_robot::LiteArm right_arm(right_config);
        int right_n = right_arm.getMotorCount();
        std::cout << "右手臂电机数量: " << right_n << std::endl;

        std::cout << "\n初始化左手臂..." << std::endl;
        litearm_robot::LiteArm left_arm(left_config);
        int left_n = left_arm.getMotorCount();
        std::cout << "左手臂电机数量: " << left_n << std::endl;

        // 右手臂角度（度 -> 弧度）
        std::vector<double> right_start = {
            degToRad(11), degToRad(-3), degToRad(0), degToRad(42),
            degToRad(0), degToRad(2), degToRad(51)
        };
        std::vector<double> right_end = {
            degToRad(40), degToRad(-3.5), degToRad(5), degToRad(62),
            degToRad(-5.5), degToRad(3.53), degToRad(2)
        };

        // 左手臂角度（度 -> 弧度）
        std::vector<double> left_start = {
            degToRad(-16), degToRad(-2.2), degToRad(-1.6), degToRad(-50),
            degToRad(0), degToRad(-2), degToRad(-30)
        };
        std::vector<double> left_end = {
            degToRad(-60), degToRad(2), degToRad(-6), degToRad(-36),
            degToRad(-5), degToRad(-6), degToRad(-1.5)
        };

        // 速度和力矩设置
        std::vector<double> velocity = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        std::vector<double> max_torque = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0};

        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "角度设置完成" << std::endl;
        std::cout << "右手臂起始角度: [11, -3, 0, 42, 0, 2, 51] (度)" << std::endl;
        std::cout << "右手臂终点角度: [40, -3.5, 5, 62, -5.5, 3.53, 2] (度)" << std::endl;
        std::cout << "左手臂起始角度: [-16, -2.2, -1.6, -50, 0, -2, -30] (度)" << std::endl;
        std::cout << "左手臂终点角度: [-60, 2, -6, -36, -5, -6, -1.5] (度)" << std::endl;
        std::cout << "速度: " << velocity[0] << " rad/s, 力矩: " << max_torque[0] << " Nm" << std::endl;
        std::cout << std::string(60, '-') << std::endl;

        // 先让手臂运动到起始位置
        std::cout << "\n正在移动到起始位置..." << std::endl;

        right_arm.posVelMaxTorque(right_start, velocity, max_torque);
        left_arm.posVelMaxTorque(left_start, velocity, max_torque);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::cout << "已到达起始位置，开始循环运动..." << std::endl;

        int cycle_count = 0;
        bool go_to_end = true;

        while (keep_running) {
            cycle_count++;
            std::cout << "\n========== 循环 #" << cycle_count << " ==========" << std::endl;

            if (go_to_end) {
                std::cout << "-> 移动到终点位置" << std::endl;
                right_arm.posVelMaxTorque(right_end, velocity, max_torque);
                left_arm.posVelMaxTorque(left_end, velocity, max_torque);
            } else {
                std::cout << "<- 移动到起始位置" << std::endl;
                right_arm.posVelMaxTorque(right_start, velocity, max_torque);
                left_arm.posVelMaxTorque(left_start, velocity, max_torque);
            }

            // 等待一小段时间后发送下一个目标
            std::this_thread::sleep_for(std::chrono::seconds(5));

            // 打印当前状态
            printCurrentAngles(right_arm, "右手臂", right_n);
            printCurrentAngles(left_arm, "左手臂", left_n);

            go_to_end = !go_to_end;
        }

        std::cout << "\n老化测试已停止，共完成 " << cycle_count << " 个循环" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}