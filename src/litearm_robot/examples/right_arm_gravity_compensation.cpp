/**
 * @file right_arm_gravity_compensation.cpp
 * @brief 右臂纯重力补偿 - 让手臂在重力作用下自由飘浮
 *
 * 工作原理：
 *   1. 读取当前7个关节角度 q
 *   2. 用 Pinocchio RNEA 算法计算当前姿态下的重力力矩 G(q)
 *   3. 将 G(q) 作为前馈力矩发送给电机（位置=0, 速度=0, Kp=0, Kd=0）
 *   4. 电机只输出对抗重力的力矩，手臂可以轻松被拖动
 *
 * 使用方法：
 *   ros2 run litearm_robot right_arm_gravity_compensation
 *   或指定配置文件和URDF：
 *   ros2 run litearm_robot right_arm_gravity_compensation <config.yaml> <right_arm.urdf>
 */

#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <cmath>
#include <algorithm>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        keep_running = 0;
        std::cout << "\n\n程序被中断，电机即将掉电，请注意安全！" << std::endl;
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

std::vector<double> clipTorque(const std::vector<double>& torque,
                                const std::vector<double>& max_torque)
{
    std::vector<double> clipped(torque.size());
    for (size_t i = 0; i < torque.size(); ++i) {
        clipped[i] = std::max(-max_torque[i], std::min(max_torque[i], torque[i]));
    }
    return clipped;
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

        std::cout << "配置文件: " << config_path << std::endl;
        std::cout << "URDF文件: " << urdf_path << std::endl;

        // 初始化机械臂
        litearm_robot::LiteArm robot(config_path);
        int n = robot.getMotorCount();
        std::cout << "电机数量: " << n << std::endl;

        // 加载 URDF 模型
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::Data data(model);

        // URDF中 base_link 是固定的，所以 nq = 7（7个旋转关节）
        std::cout << "Pinocchio 模型: nq=" << model.nq << ", nv=" << model.nv
                  << ", njoints=" << model.njoints << std::endl;

        if (model.nq != n) {
            std::cerr << "警告: Pinocchio nq=" << model.nq << " != 电机数量 " << n
                      << ", 请检查URDF文件" << std::endl;
        }

        // 力矩安全限幅（根据电机规格设置）
        // motor type: 5047_36 -> ~21Nm, 6056_36 -> ~36Nm, 4438_30 -> ~6Nm
        std::vector<double> tau_limit = {15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0};

        // 重力补偿力矩增益系数（各关节独立调整）
        // joint4 计算力矩偏大，需要调小；其他关节暂时设为1.0
        std::vector<double> gravity_gain = {1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0};

        // 零位置、零速度、零增益（纯力矩控制）
        std::vector<double> zero_pos(n, 0.0);
        std::vector<double> zero_vel(n, 0.0);
        std::vector<double> zero_kp(n, 0.0);
        std::vector<double> zero_kd(n, 0.0);

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "右臂纯重力补偿模式" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "力矩限幅: [";
        for (int i = 0; i < n; ++i) {
            std::cout << tau_limit[i] << (i < n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        std::cout << "重力增益: [";
        for (int i = 0; i < n; ++i) {
            std::cout << gravity_gain[i] << (i < n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        std::cout << "\n提示：" << std::endl;
        std::cout << "- 机械臂将补偿自身重力，可以轻松拖动" << std::endl;
        std::cout << "- 按 Ctrl+C 退出" << std::endl;
        std::cout << "- 退出后电机会掉电，请注意安全！" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;

        // 先读取当前位置，确认通信正常
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

        std::cout << "\n开始重力补偿...\n" << std::endl;

        auto last_print_time = std::chrono::steady_clock::now();
        const double print_interval = 0.5;
        int loop_count = 0;

        while (keep_running) {
            // 1. 读取当前关节状态
            robot.send_get_motor_state_cmd();
            robot.motor_send_cmd();

            auto q_current = robot.getCurrentPos();
            auto v_current = robot.getCurrentVel();

            // 2. 计算重力补偿力矩
            std::vector<double> G = computeGravityTorque(model, data, q_current);

            // 只取前 n 个关节
            G.resize(n);

            // 应用重力补偿增益系数
            for (int i = 0; i < n; ++i) {
                G[i] *= gravity_gain[i];
            }

            // 3. 力矩限幅
            std::vector<double> tau_cmd = clipTorque(G, tau_limit);

            // 4. 发送力矩命令（pos=0, vel=0, kp=0, kd=0, torque=G(q)）
            robot.posVelTorqueKpKd(zero_pos, zero_vel, tau_cmd, zero_kp, zero_kd);

            // 5. 定期打印状态
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_print_time).count();
            if (elapsed >= print_interval) {
                std::cout << std::fixed << std::setprecision(3);
                std::cout << "\n--- 循环 #" << loop_count << " ---" << std::endl;

                std::cout << "关节角度(deg): [";
                for (int i = 0; i < n; ++i) {
                    std::cout << std::setw(8) << (q_current[i] * 180.0 / M_PI)
                              << (i < n-1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "重力力矩(Nm):  [";
                for (int i = 0; i < n; ++i) {
                    std::cout << std::setw(8) << G[i]
                              << (i < n-1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "实际输出(Nm):  [";
                for (int i = 0; i < n; ++i) {
                    std::cout << std::setw(8) << tau_cmd[i]
                              << (i < n-1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                last_print_time = now;
            }

            loop_count++;

            // 200Hz 控制频率
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        std::cout << "\n重力补偿已停止，电机将掉电。" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
