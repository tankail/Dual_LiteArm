/**
 * @file right_arm_gravity_calc.cpp
 * @brief 右臂重力补偿计算器（仅打印，不输出力矩）
 *
 * 安全诊断工具 - 只读取关节位置，计算重力补偿力矩并打印到终端
 * 不会向电机发送任何命令，确保机械臂安全
 *
 * 使用方法：
 *   ros2 run litearm_robot right_arm_gravity_calc
 */

#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <cmath>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        keep_running = 0;
        std::cout << "\n\n程序退出" << std::endl;
    }
}

std::vector<double> computeGravityTorque(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const std::vector<double>& q)
{
    Eigen::VectorXd q_eigen(model.nq);
    for (int i = 0; i < model.nq; ++i) {
        q_eigen[i] = (i < (int)q.size()) ? q[i] : 0.0;
    }

    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model.nv);

    Eigen::VectorXd tau = pinocchio::rnea(model, data, q_eigen, v_zero, a_zero);

    std::vector<double> gravity_torque(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
        gravity_torque[i] = tau[i];
    }
    return gravity_torque;
}

void printTorqueTable(const std::vector<double>& q, const std::vector<double>& G, int n)
{
    std::cout << "\n+------+----------+-----------+-------------+\n";
    std::cout << "| 关节 | 角度(°)  | 重力力矩  | 单位       |\n";
    std::cout << "+------+----------+-----------+-------------+\n";

    const char* joint_names[] = {
        "joint1", "joint2", "joint3",
        "joint4", "joint5", "joint6", "joint7"
    };

    for (int i = 0; i < n; ++i) {
        double angle_deg = q[i] * 180.0 / M_PI;
        std::cout << "| " << std::setw(5) << joint_names[i]
                  << " | " << std::fixed << std::setprecision(2) << std::setw(8) << angle_deg
                  << " | " << std::setprecision(4) << std::setw(9) << G[i]
                  << " | Nm         |\n";
    }
    std::cout << "+------+----------+-----------+-------------+\n";

    // 打印总计
    double total = 0.0;
    for (int i = 0; i < n; ++i) total += std::abs(G[i]);
    std::cout << "| 合计 |          | " << std::setprecision(4) << std::setw(9) << total
              << " | Nm (|G|总和) |\n";
    std::cout << "+------+----------+-----------+-------------+\n";
}

int main(int argc, char** argv)
{
    try {
        signal(SIGINT, signal_handler);

        // 配置文件路径
        std::string config_path = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_right_arm.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        // URDF 路径（仅右臂）
        std::string urdf_path = ament_index_cpp::get_package_share_directory("litearm_robot")
            + "/urdf/LiteArm_A10_251224_right_arm.urdf";
        if (argc > 2) {
            urdf_path = argv[2];
        }

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "右臂重力补偿计算器（仅诊断，不输出力矩）" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "配置文件: " << config_path << std::endl;
        std::cout << "URDF文件: " << urdf_path << std::endl;

        // 初始化机械臂（只读取状态，不输出力矩）
        litearm_robot::LiteArm robot(config_path);
        int n = robot.getMotorCount();
        std::cout << "电机数量: " << n << std::endl;

        // 加载 URDF 模型
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::Data data(model);

        std::cout << "Pinocchio 模型: nq=" << model.nq << ", nv=" << model.nv
                  << ", njoints=" << model.njoints << std::endl;

        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "【安全提示】本程序仅读取关节位置并计算重力补偿力矩" << std::endl;
        std::cout << "           不会向电机发送任何命令，机械臂处于自由状态" << std::endl;
        std::cout << std::string(60, '-') << std::endl;

        // 先读取当前位置
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto init_pos = robot.getCurrentPos();

        std::cout << "\n初始位置(rad): [";
        for (int i = 0; i < n; ++i) {
            std::cout << std::fixed << std::setprecision(3) << init_pos[i]
                      << (i < n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        auto last_print_time = std::chrono::steady_clock::now();
        const double print_interval = 1.0;  // 每秒打印一次
        int sample_count = 0;

        std::cout << "\n开始读取关节状态并计算重力补偿...\n" << std::endl;
        std::cout << "提示：移动手臂到不同姿态，观察重力力矩变化\n" << std::endl;

        while (keep_running) {
            // 1. 读取当前关节状态
            robot.send_get_motor_state_cmd();
            robot.motor_send_cmd();

            auto q_current = robot.getCurrentPos();
            auto v_current = robot.getCurrentVel();

            // 2. 计算重力补偿力矩
            std::vector<double> G = computeGravityTorque(model, data, q_current);
            G.resize(n);

            // 3. 打印状态
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_print_time).count();
            if (elapsed >= print_interval) {
                sample_count++;
                std::cout << "\n========== 样本 #" << sample_count << " ==========" << std::endl;
                printTorqueTable(q_current, G, n);
                last_print_time = now;
            }

            // 50Hz 读取频率（快速采样，但慢速打印）
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
