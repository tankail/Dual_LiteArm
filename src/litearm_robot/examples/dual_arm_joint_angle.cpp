/**
 * @file dual_arm_joint_angle.cpp
 * @brief 双臂关节角度读取器 - 直接通过SDK读取并在终端显示（度）
 *
 * 使用方法:
 *   ros2 run litearm_robot dual_arm_joint_angle              # 显示双臂
 *   ros2 run litearm_robot dual_arm_joint_angle --left       # 只显示左臂
 *   ros2 run litearm_robot dual_arm_joint_angle --right      # 只显示右臂
 */

#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>
#include <algorithm>
#include <cstring>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        keep_running = false;
    }
}

void clearScreen() {
    std::cout << "\033[2J\033[H" << std::flush;  // 清屏
}

std::string makeBar(double value, double vmin, double vmax, int width = 20) {
    value = std::max(vmin, std::min(vmax, value));
    int pos = static_cast<int>((value - vmin) / (vmax - vmin) * width);
    std::string bar = "[";
    for (int i = 0; i < width; i++) {
        bar += (i == pos) ? "*" : (i == width / 2 ? "|" : " ");
    }
    bar += "]";
    return bar;
}

void readAngles(litearm_robot::LiteArm& robot, std::vector<double>& pos,
                std::vector<double>& vel, std::vector<double>& tor, int n) {
    robot.send_get_motor_state_cmd();
    robot.motor_send_cmd();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    pos = robot.getCurrentPos();
    vel = robot.getCurrentVel();
    tor = robot.getCurrentTorque();
}

void printArmTable(const std::string& title, litearm_robot::LiteArm& robot,
                   int n, const char* joint_names[]) {
    std::vector<double> pos, vel, tor;
    readAngles(robot, pos, vel, tor, n);

    std::cout << "\n\033[1;36m  [" << title << "]\033[0m" << std::endl;
    std::cout << "  " << std::string(68, '-') << std::endl;
    std::cout << "  " << std::left << std::setw(18) << "关节"
              << std::setw(12) << "角度(°)"
              << std::setw(14) << "速度(°/s)"
              << std::setw(12) << "力矩(Nm)"
              << "  位置指示" << std::endl;
    std::cout << "  " << std::string(68, '-') << std::endl;

    for (int i = 0; i < n; i++) {
        double deg = pos[i] * 180.0 / M_PI;
        double vel_deg = vel[i] * 180.0 / M_PI;
        std::string bar = makeBar(deg, -180, 180);

        std::cout << "  " << std::left << std::setw(18) << joint_names[i]
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(8) << deg
                  << std::setw(12) << vel_deg
                  << std::setw(10) << tor[i]
                  << "  " << bar << std::endl;
    }
    std::cout << "  " << std::string(68, '-') << std::endl;
}

int main(int argc, char** argv) {
    try {
        signal(SIGINT, signal_handler);

        // 解析参数
        bool show_left = true, show_right = true;
        for (int i = 1; i < argc; i++) {
            if (std::strcmp(argv[i], "--left") == 0) show_right = false;
            if (std::strcmp(argv[i], "--right") == 0) show_left = false;
        }

        std::string right_config = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_right_arm.yaml";
        std::string left_config = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_left_arm.yaml";

        // 初始化
        litearm_robot::LiteArm* right_arm = nullptr;
        litearm_robot::LiteArm* left_arm = nullptr;
        int right_n = 0, left_n = 0;

        if (show_right) {
            std::cout << "初始化右臂..." << std::endl;
            right_arm = new litearm_robot::LiteArm(right_config);
            right_n = right_arm->getMotorCount();
            std::cout << "右臂电机数: " << right_n << std::endl;
        }

        if (show_left) {
            std::cout << "初始化左臂..." << std::endl;
            left_arm = new litearm_robot::LiteArm(left_config);
            left_n = left_arm->getMotorCount();
            std::cout << "左臂电机数: " << left_n << std::endl;
        }

        std::cout << "\n开始读取关节角度... (Ctrl+C 退出)\n" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        const char* right_joints[] = {
            "r_joint1_joint", "r_joint2_joint", "r_joint3_joint",
            "r_joint4_joint", "r_joint5_joint", "r_joint6_joint", "r_joint7_joint"
        };
        const char* left_joints[] = {
            "l_joint1_joint", "l_joint2_joint", "l_joint3_joint",
            "l_joint4_joint", "l_joint5_joint", "l_joint6_joint", "l_joint7_joint"
        };

        int sample = 0;
        auto last_time = std::chrono::steady_clock::now();

        while (keep_running) {
            clearScreen();

            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_time).count();
            double freq = (elapsed > 0) ? 1.0 / elapsed : 0;
            last_time = now;
            sample++;

            std::cout << "\033[1;37m"
                      << "======================================================================"
                      << "\033[0m" << std::endl;
            std::cout << "\033[1;37m"
                      << "  LiteArm 双臂关节角度显示器 (SDK直接读取)"
                      << "\033[0m" << std::endl;
            std::cout << "  Sample #" << sample
                      << "  |  刷新频率: " << std::fixed << std::setprecision(1)
                      << freq << " Hz"
                      << "  |  按 Ctrl+C 退出" << std::endl;
            std::cout << "\033[1;37m"
                      << "======================================================================"
                      << "\033[0m" << std::endl;

            if (right_arm) {
                printArmTable("右臂", *right_arm, right_n, right_joints);
            }

            if (left_arm) {
                printArmTable("左臂", *left_arm, left_n, left_joints);
            }

            std::cout << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        // 清理
        if (right_arm) delete right_arm;
        if (left_arm) delete left_arm;

        std::cout << "\n\n程序退出, 共采集 " << sample << " 个样本" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
