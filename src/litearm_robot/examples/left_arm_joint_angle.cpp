/**
 * @file left_arm_joint_angle.cpp
 * @brief 左臂关节角度读取器
 *
 * 读取左手臂每个关节的角度并输出到终端
 *
 * 使用方法：
 *   ros2 run litearm_robot left_arm_joint_angle
 */

#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>

volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        keep_running = 0;
        std::cout << "\n\n程序退出" << std::endl;
    }
}

void printJointTable(const std::vector<double>& angles, int n)
{
    std::cout << "\n+------+------------+-----------+------------+\n";
    std::cout << "| 关节 | 角度(rad)  | 角度(°)   | 状态       |\n";
    std::cout << "+------+------------+-----------+------------+\n";

    const char* joint_names[] = {
        "joint1", "joint2", "joint3",
        "joint4", "joint5", "joint6", "joint7"
    };

    for (int i = 0; i < n; ++i) {
        double angle_deg = angles[i] * 180.0 / M_PI;
        std::cout << "| " << std::setw(5) << joint_names[i]
                  << " | " << std::fixed << std::setprecision(4) << std::setw(10) << angles[i]
                  << " | " << std::setprecision(2) << std::setw(8) << angle_deg
                  << " | 正常       |\n";
    }
    std::cout << "+------+------------+-----------+------------+\n";
}

int main(int argc, char** argv)
{
    try {
        signal(SIGINT, signal_handler);

        // 配置文件路径（左手臂）
        std::string config_path = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_left_arm.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        // URDF 路径（左手臂）
        std::string urdf_path = ament_index_cpp::get_package_share_directory("litearm_robot")
            + "/urdf/LiteArm_A10_251224_left_arm.urdf";
        if (argc > 2) {
            urdf_path = argv[2];
        }

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "左臂关节角度读取器" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "配置文件: " << config_path << std::endl;
        std::cout << "URDF文件: " << urdf_path << std::endl;

        // 初始化机械臂
        litearm_robot::LiteArm robot(config_path);
        int n = robot.getMotorCount();
        std::cout << "电机数量: " << n << std::endl;

        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "开始读取左臂关节角度...\n" << std::endl;

        // 先读取当前位置
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto init_pos = robot.getCurrentPos();

        std::cout << "初始位置(rad): [";
        for (int i = 0; i < n; ++i) {
            std::cout << std::fixed << std::setprecision(3) << init_pos[i]
                      << (i < n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        auto last_print_time = std::chrono::steady_clock::now();
        const double print_interval = 1.0;  // 每秒打印一次
        int sample_count = 0;

        std::cout << "\n提示：移动手臂到不同姿态，观察角度变化\n" << std::endl;

        while (keep_running) {
            // 读取当前关节状态
            robot.send_get_motor_state_cmd();
            robot.motor_send_cmd();

            auto q_current = robot.getCurrentPos();

            // 打印状态
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_print_time).count();
            if (elapsed >= print_interval) {
                sample_count++;
                std::cout << "\n========== 样本 #" << sample_count << " ==========" << std::endl;
                printJointTable(q_current, n);
                last_print_time = now;
            }

            // 50Hz 读取频率
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}