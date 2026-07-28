/**
 * @file dual_arm_gravity_compensation.cpp
 * @brief 双臂纯重力补偿 - 同时启动左臂和右臂重力补偿
 *
 * 工作原理与 left/right_arm_gravity_compensation.cpp 保持一致：
 *   1. 左臂使用 ttyACM0 对应的 litearm_left_arm.yaml
 *   2. 右臂使用 ttyACM1 对应的 litearm_right_arm.yaml
 *   3. 每个控制周期读取左右臂当前7个关节角度
 *   4. 分别用左右臂 URDF 通过 Pinocchio RNEA 计算 G(q)
 *   5. 发送 pos=0, vel=0, torque=G(q), kp=0, kd=0 的纯前馈力矩
 *
 * 使用方法：
 *   ros2 run litearm_robot dual_arm_gravity_compensation
 *   或指定配置文件和URDF：
 *   ros2 run litearm_robot dual_arm_gravity_compensation \
 *     <left_config.yaml> <right_config.yaml> <left_arm.urdf> <right_arm.urdf>
 */

#include "litearm_robot/LiteArm.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM) {
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
        q_eigen[i] = (i < static_cast<int>(q.size())) ? q[i] : 0.0;
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

void applyGain(std::vector<double>& torque, const std::vector<double>& gain)
{
    for (size_t i = 0; i < torque.size() && i < gain.size(); ++i) {
        torque[i] *= gain[i];
    }
}

void printVector(const std::string& label, const std::vector<double>& values, int n)
{
    std::cout << label << " [";
    for (int i = 0; i < n; ++i) {
        std::cout << std::fixed << std::setprecision(3) << values[i]
                  << (i < n - 1 ? ", " : "");
    }
    std::cout << "]" << std::endl;
}

struct ArmContext
{
    std::string side;
    std::string config_path;
    std::string urdf_path;
    std::vector<double> tau_limit;
    std::vector<double> gravity_gain;
    litearm_robot::LiteArm robot;
    int n;
    pinocchio::Model model;
    pinocchio::Data data;
    std::vector<double> zero_pos;
    std::vector<double> zero_vel;
    std::vector<double> zero_kp;
    std::vector<double> zero_kd;

    ArmContext(const std::string& side_name,
               const std::string& config,
               const std::string& urdf,
               const std::vector<double>& limits,
               const std::vector<double>& gains)
        : side(side_name),
          config_path(config),
          urdf_path(urdf),
          tau_limit(limits),
          gravity_gain(gains),
          robot(config_path),
          n(robot.getMotorCount())
    {
        pinocchio::urdf::buildModel(urdf_path, model);
        data = pinocchio::Data(model);

        zero_pos.assign(n, 0.0);
        zero_vel.assign(n, 0.0);
        zero_kp.assign(n, 0.0);
        zero_kd.assign(n, 0.0);

        if (model.nq != n) {
            std::cerr << "[" << side << "] 警告: Pinocchio nq=" << model.nq
                      << " != 电机数量 " << n << "，请检查URDF文件" << std::endl;
        }
        if (static_cast<int>(tau_limit.size()) < n ||
            static_cast<int>(gravity_gain.size()) < n) {
            throw std::runtime_error("力矩限幅或重力增益长度小于电机数量: " + side);
        }
    }

    void printInfo() const
    {
        std::cout << "\n[" << side << "] 配置文件: " << config_path << std::endl;
        std::cout << "[" << side << "] URDF文件: " << urdf_path << std::endl;
        std::cout << "[" << side << "] 电机数量: " << n << std::endl;
        std::cout << "[" << side << "] Pinocchio 模型: nq=" << model.nq
                  << ", nv=" << model.nv << ", njoints=" << model.njoints
                  << std::endl;
        printVector("[" + side + "] 力矩限幅:", tau_limit, n);
        printVector("[" + side + "] 重力增益:", gravity_gain, n);
    }

    std::vector<double> readPositions()
    {
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        return robot.getCurrentPos();
    }

    std::vector<double> computeCommand(const std::vector<double>& q)
    {
        std::vector<double> G = computeGravityTorque(model, data, q);
        G.resize(n);
        applyGain(G, gravity_gain);
        return clipTorque(G, tau_limit);
    }

    bool sendTorque(const std::vector<double>& tau_cmd)
    {
        return robot.posVelTorqueKpKd(zero_pos, zero_vel, tau_cmd, zero_kp, zero_kd);
    }
};

int main(int argc, char** argv)
{
    try {
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        std::string left_config;
        std::string right_config;
        std::string left_urdf;
        std::string right_urdf;
        if (argc > 4) {
            // The backend passes absolute paths, so no local ament overlay is
            // required when this executable is started from the web server.
            left_config = argv[1];
            right_config = argv[2];
            left_urdf = argv[3];
            right_urdf = argv[4];
        } else {
            left_config = ament_index_cpp::get_package_share_directory("litearm_config")
                + "/robot_param/litearm_left_arm.yaml";
            right_config = ament_index_cpp::get_package_share_directory("litearm_config")
                + "/robot_param/litearm_right_arm.yaml";
            left_urdf = ament_index_cpp::get_package_share_directory("litearm_robot")
                + "/urdf/LiteArm_A10_251224_left_arm.urdf";
            right_urdf = ament_index_cpp::get_package_share_directory("litearm_robot")
                + "/urdf/LiteArm_A10_251224_right_arm.urdf";
        }

        const std::vector<double> tau_limit = {15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0};
        const std::vector<double> left_gain = {0.85, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0};
        const std::vector<double> right_gain = {1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0};

        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << "双臂纯重力补偿模式" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "左臂端口由 litearm_left_arm.yaml 指定，默认 /dev/ttyACM0" << std::endl;
        std::cout << "右臂端口由 litearm_right_arm.yaml 指定，默认 /dev/ttyACM1" << std::endl;

        ArmContext left("left", left_config, left_urdf, tau_limit, left_gain);
        ArmContext right("right", right_config, right_urdf, tau_limit, right_gain);

        left.printInfo();
        right.printInfo();

        left.robot.send_get_motor_state_cmd();
        right.robot.send_get_motor_state_cmd();
        left.robot.motor_send_cmd();
        right.robot.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto left_init = left.robot.getCurrentPos();
        auto right_init = right.robot.getCurrentPos();
        printVector("[left] 初始位置(rad):", left_init, left.n);
        printVector("[right] 初始位置(rad):", right_init, right.n);

        std::cout << "\n提示：" << std::endl;
        std::cout << "- 左右臂将同时进入纯重力补偿，可以轻松拖动" << std::endl;
        std::cout << "- 按 Ctrl+C 退出" << std::endl;
        std::cout << "- 退出后电机会掉电，请注意安全" << std::endl;
        std::cout << "\n开始双臂重力补偿...\n" << std::endl;

        auto last_print_time = std::chrono::steady_clock::now();
        auto next_tick = std::chrono::steady_clock::now();
        const auto loop_period = std::chrono::milliseconds(5);  // 200 Hz
        const double print_interval = 0.5;
        int loop_count = 0;

        while (keep_running) {
            next_tick += loop_period;

            auto left_q = left.readPositions();
            auto right_q = right.readPositions();

            auto left_tau = left.computeCommand(left_q);
            auto right_tau = right.computeCommand(right_q);

            left.sendTorque(left_tau);
            right.sendTorque(right_tau);

            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_print_time).count();
            if (elapsed >= print_interval) {
                std::cout << std::fixed << std::setprecision(3);
                std::cout << "\n--- 双臂重力补偿循环 #" << loop_count << " ---" << std::endl;
                printVector("[left] 关节角度(rad): ", left_q, left.n);
                printVector("[left] 输出力矩(Nm):  ", left_tau, left.n);
                printVector("[right] 关节角度(rad):", right_q, right.n);
                printVector("[right] 输出力矩(Nm): ", right_tau, right.n);
                last_print_time = now;
            }

            loop_count++;
            std::this_thread::sleep_until(next_tick);
            if (std::chrono::steady_clock::now() > next_tick + loop_period) {
                next_tick = std::chrono::steady_clock::now();
            }
        }

        std::cout << "\n双臂重力补偿已停止，电机将掉电。" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
