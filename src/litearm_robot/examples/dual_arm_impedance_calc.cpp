/**
 * @file dual_arm_impedance_calc.cpp
 * @brief 双臂关节空间阻抗力矩计算器（仅打印，不输出MIT控制力矩）
 *
 * 安全诊断工具：
 *   1. 启动后读取左右臂当前位置，并锁存为 q_target
 *   2. 循环读取当前 q 和 dq
 *   3. 计算 tau = G(q) + Kp * (q_target - q) - Kd * dq
 *   4. 打印重力项、PD项、总力矩和限幅后力矩
 *
 * 本程序不会调用 posVelTorqueKpKd()，不会向电机发送MIT力矩控制命令。
 *
 * 使用方法：
 *   ros2 run litearm_robot dual_arm_impedance_calc
 *   或指定配置文件和URDF：
 *   ros2 run litearm_robot dual_arm_impedance_calc \
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
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM) {
        keep_running = 0;
        std::cout << "\n\n程序退出" << std::endl;
    }
}

std::vector<double> computeGravityTorque(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const std::vector<double>& q)
{
    Eigen::VectorXd q_eigen = Eigen::VectorXd::Zero(model.nq);
    for (int i = 0; i < model.nq && i < static_cast<int>(q.size()); ++i) {
        q_eigen[i] = q[i];
    }

    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q_eigen, v_zero, a_zero);

    std::vector<double> gravity(q.size(), 0.0);
    for (size_t i = 0; i < gravity.size() && i < static_cast<size_t>(tau.size()); ++i) {
        gravity[i] = tau[static_cast<Eigen::Index>(i)];
    }
    return gravity;
}

std::vector<double> clipTorque(
    const std::vector<double>& torque,
    const std::vector<double>& max_torque)
{
    std::vector<double> clipped(torque.size(), 0.0);
    for (size_t i = 0; i < torque.size(); ++i) {
        const double limit = i < max_torque.size() ? std::max(0.0, max_torque[i]) : 0.0;
        clipped[i] = std::max(-limit, std::min(limit, torque[i]));
    }
    return clipped;
}

bool validState(
    const std::vector<double>& position,
    const std::vector<double>& velocity,
    int expected_size)
{
    if (static_cast<int>(position.size()) < expected_size ||
        static_cast<int>(velocity.size()) < expected_size) {
        return false;
    }
    for (int i = 0; i < expected_size; ++i) {
        if (!std::isfinite(position[i]) || !std::isfinite(velocity[i]) ||
            std::abs(position[i]) >= 100.0 || std::abs(velocity[i]) >= 100.0) {
            return false;
        }
    }
    return true;
}

void printVector(const std::string& label, const std::vector<double>& values, int n)
{
    std::cout << label << " [";
    for (int i = 0; i < n; ++i) {
        std::cout << std::fixed << std::setprecision(4) << values[i]
                  << (i + 1 < n ? ", " : "");
    }
    std::cout << "]" << std::endl;
}

void printTorqueTable(
    const std::string& side,
    const std::vector<double>& q_target,
    const std::vector<double>& q,
    const std::vector<double>& dq,
    const std::vector<double>& gravity,
    const std::vector<double>& pd,
    const std::vector<double>& total,
    const std::vector<double>& clipped,
    int n)
{
    std::cout << "\n[" << side << "] tau = G(q) + Kp(q_target-q) - Kd*dq\n";
    std::cout << "+------+----------+----------+----------+----------+----------+----------+----------+\n";
    std::cout << "| 关节 | q(deg)   | target   | error    | dq       | G(Nm)    | PD(Nm)   | tau(Nm)  |\n";
    std::cout << "+------+----------+----------+----------+----------+----------+----------+----------+\n";

    for (int i = 0; i < n; ++i) {
        const double error = q_target[i] - q[i];
        std::cout << "| j" << std::left << std::setw(4) << (i + 1) << std::right
                  << " | " << std::fixed << std::setprecision(2) << std::setw(8)
                  << (q[i] * 180.0 / M_PI)
                  << " | " << std::setprecision(4) << std::setw(8) << q_target[i]
                  << " | " << std::setw(8) << error
                  << " | " << std::setw(8) << dq[i]
                  << " | " << std::setw(8) << gravity[i]
                  << " | " << std::setw(8) << pd[i]
                  << " | " << std::setw(8) << clipped[i]
                  << " |\n";
    }
    std::cout << "+------+----------+----------+----------+----------+----------+----------+----------+\n";

    double sum_abs_total = 0.0;
    double sum_abs_clipped = 0.0;
    for (int i = 0; i < n; ++i) {
        sum_abs_total += std::abs(total[i]);
        sum_abs_clipped += std::abs(clipped[i]);
    }
    std::cout << "| |tau raw| sum = " << std::fixed << std::setprecision(4)
              << sum_abs_total << " Nm, |tau clipped| sum = "
              << sum_abs_clipped << " Nm |\n";
}

struct ArmCalcContext
{
    std::string side;
    litearm_robot::LiteArm robot;
    int n;
    pinocchio::Model model;
    pinocchio::Data data;
    std::vector<double> q_target;
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> gravity_gain;
    std::vector<double> torque_limit;

    ArmCalcContext(
        const std::string& side_name,
        const std::string& config_path,
        const std::string& urdf_path,
        const std::vector<double>& gravity_gain_in,
        const std::vector<double>& kp_in,
        const std::vector<double>& kd_in,
        const std::vector<double>& torque_limit_in)
        : side(side_name),
          robot(config_path),
          n(robot.getMotorCount()),
          model(),
          data(model),
          q_target(n, 0.0),
          kp(kp_in),
          kd(kd_in),
          gravity_gain(gravity_gain_in),
          torque_limit(torque_limit_in)
    {
        pinocchio::urdf::buildModel(urdf_path, model);
        data = pinocchio::Data(model);
        if (model.nq != n) {
            std::cerr << "[" << side << "] 警告: Pinocchio nq=" << model.nq
                      << " != 电机数量 " << n << "，请检查URDF文件" << std::endl;
        }
        if (static_cast<int>(kp.size()) < n ||
            static_cast<int>(kd.size()) < n ||
            static_cast<int>(gravity_gain.size()) < n ||
            static_cast<int>(torque_limit.size()) < n) {
            throw std::runtime_error("参数长度小于电机数量: " + side);
        }
    }

    std::vector<double> readPosition()
    {
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        return robot.getCurrentPos();
    }

    std::vector<double> readVelocity()
    {
        return robot.getCurrentVel();
    }

    void latchCurrentTarget()
    {
        auto q = readPosition();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        q = robot.getCurrentPos();
        auto dq = robot.getCurrentVel();
        if (!validState(q, dq, n)) {
            throw std::runtime_error("无法读取有效初始关节状态: " + side);
        }
        q_target = q;
        q_target.resize(n, 0.0);
    }

    void compute(
        const std::vector<double>& q,
        const std::vector<double>& dq,
        std::vector<double>& gravity,
        std::vector<double>& pd,
        std::vector<double>& total,
        std::vector<double>& clipped)
    {
        gravity = computeGravityTorque(model, data, q);
        gravity.resize(n, 0.0);
        pd.assign(n, 0.0);
        total.assign(n, 0.0);

        for (int i = 0; i < n; ++i) {
            gravity[i] *= gravity_gain[i];
            pd[i] = kp[i] * (q_target[i] - q[i]) - kd[i] * dq[i];
            total[i] = gravity[i] + pd[i];
        }
        clipped = clipTorque(total, torque_limit);
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

        const std::vector<double> torque_limit =
            {15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0};
        const std::vector<double> left_gravity_gain =
            {0.85, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0};
        const std::vector<double> right_gravity_gain =
            {1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0};
        const std::vector<double> kp =
            {4.0, 8.0, 8.0, 3.0, 2.0, 1.0, 0.8};
        const std::vector<double> kd =
            {0.6, 0.8, 0.8, 0.4, 0.25, 0.15, 0.1};

        std::cout << "\n" << std::string(72, '=') << std::endl;
        std::cout << "双臂阻抗力矩计算器（仅打印，不输出MIT控制力矩）" << std::endl;
        std::cout << "tau = G(q) + Kp(q_target-q) - Kd*dq" << std::endl;
        std::cout << std::string(72, '=') << std::endl;
        std::cout << "左臂配置: " << left_config << std::endl;
        std::cout << "右臂配置: " << right_config << std::endl;
        std::cout << "左臂URDF: " << left_urdf << std::endl;
        std::cout << "右臂URDF: " << right_urdf << std::endl;
        printVector("Kp: ", kp, static_cast<int>(kp.size()));
        printVector("Kd: ", kd, static_cast<int>(kd.size()));
        printVector("Torque limit: ", torque_limit, static_cast<int>(torque_limit.size()));

        ArmCalcContext left(
            "left", left_config, left_urdf, left_gravity_gain, kp, kd, torque_limit);
        ArmCalcContext right(
            "right", right_config, right_urdf, right_gravity_gain, kp, kd, torque_limit);

        left.latchCurrentTarget();
        right.latchCurrentTarget();

        printVector("[left] q_target(rad): ", left.q_target, left.n);
        printVector("[right] q_target(rad):", right.q_target, right.n);
        std::cout << "\n开始计算双臂阻抗力矩。移动手臂后观察 PD 项变化，Ctrl+C 退出。\n"
                  << std::endl;

        auto last_print_time = std::chrono::steady_clock::now();
        const double print_interval = 1.0;
        int sample_count = 0;

        while (keep_running) {
            auto left_q = left.readPosition();
            auto left_dq = left.readVelocity();
            auto right_q = right.readPosition();
            auto right_dq = right.readVelocity();

            if (!validState(left_q, left_dq, left.n) ||
                !validState(right_q, right_dq, right.n)) {
                std::cerr << "[安全停止] 读取到无效双臂关节状态，停止计算。" << std::endl;
                break;
            }

            std::vector<double> left_g;
            std::vector<double> left_pd;
            std::vector<double> left_total;
            std::vector<double> left_clipped;
            std::vector<double> right_g;
            std::vector<double> right_pd;
            std::vector<double> right_total;
            std::vector<double> right_clipped;

            left.compute(left_q, left_dq, left_g, left_pd, left_total, left_clipped);
            right.compute(right_q, right_dq, right_g, right_pd, right_total, right_clipped);

            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - last_print_time).count();
            if (elapsed >= print_interval) {
                ++sample_count;
                std::cout << "\n========== 样本 #" << sample_count << " ==========" << std::endl;
                printTorqueTable(
                    "left", left.q_target, left_q, left_dq,
                    left_g, left_pd, left_total, left_clipped, left.n);
                printTorqueTable(
                    "right", right.q_target, right_q, right_dq,
                    right_g, right_pd, right_total, right_clipped, right.n);
                last_print_time = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cout << "\n双臂阻抗力矩计算器已退出。" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
