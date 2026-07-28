/**
 * @file dual_arm_impedance_compensation.cpp
 * @brief 双臂关节空间阻抗控制：G(q) + Kp(qd-q) - Kd*dq
 *
 * qd 优先使用上位机在模式切换瞬间捕获并传入的当前位置。控制过程中
 * 机械臂受到外力被拖动时，会产生位置 PD 回弹力矩，同时持续补偿自身重力。
 *
 * 左臂使用 /dev/ttyACM0 对应配置，右臂使用 /dev/ttyACM1 对应配置。
 * 电机命令与已验证的 dual_arm_gravity_compensation 完全相同：
 * pos=0, vel=0, torque=tau, kp=0, kd=0。
 *
 * 也支持独立运行时不传目标位置，此时程序会在完成初始化后锁存当前位置：
 * dual_arm_impedance_compensation <left_cfg> <right_cfg> <left_urdf> <right_urdf>
 *   --left-target q1 q2 q3 q4 q5 q6 q7
 *   --right-target q1 q2 q3 q4 q5 q6 q7
 */

#include "litearm_robot/LiteArm.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cmath>
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
        std::cout << "\n\n阻抗控制被中断，电机即将掉电，请注意安全！" << std::endl;
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

void printVector(const std::string& label, const std::vector<double>& values)
{
    std::cout << label << " [";
    for (size_t i = 0; i < values.size(); ++i) {
        std::cout << std::fixed << std::setprecision(3) << values[i]
                  << (i + 1 < values.size() ? ", " : "");
    }
    std::cout << "]" << std::endl;
}

bool parseTargetArgument(
    int argc,
    char** argv,
    const std::string& option,
    std::vector<double>& target)
{
    for (int i = 5; i < argc; ++i) {
        if (option != argv[i]) {
            continue;
        }
        if (i + 7 >= argc) {
            throw std::runtime_error(option + " requires 7 joint positions");
        }
        target.clear();
        for (int j = 0; j < 7; ++j) {
            target.push_back(std::stod(argv[i + 1 + j]));
            if (!std::isfinite(target.back()) || std::abs(target.back()) >= 100.0) {
                throw std::runtime_error(option + " contains an invalid joint position");
            }
        }
        return true;
    }
    return false;
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

struct ArmContext
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

    ArmContext(
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

    void latchTarget(const std::vector<double>& target)
    {
        if (!validState(target, std::vector<double>(n, 0.0), n)) {
            throw std::runtime_error("传入的目标位置无效: " + side);
        }
        q_target = target;
        q_target.resize(n, 0.0);
    }

    std::vector<double> computeCommand(
        const std::vector<double>& q,
        const std::vector<double>& dq)
    {
        std::vector<double> gravity = computeGravityTorque(model, data, q);
        gravity.resize(n, 0.0);

        std::vector<double> pd(n, 0.0);
        std::vector<double> total(n, 0.0);
        for (int i = 0; i < n; ++i) {
            gravity[i] *= gravity_gain[i];
            pd[i] = kp[i] * (q_target[i] - q[i]) - kd[i] * dq[i];
            total[i] = gravity[i] + pd[i];
        }
        return clipTorque(total, torque_limit);
    }

    bool sendCommand(const std::vector<double>& torque)
    {
        const std::vector<double> zero_position(n, 0.0);
        const std::vector<double> zero_velocity(n, 0.0);
        const std::vector<double> zero_kp(n, 0.0);
        const std::vector<double> zero_kd(n, 0.0);
        // Software computes the complete impedance torque. MIT's internal
        // Kp/Kd are zero; torque is sent as the feed-forward term.
        return robot.posVelTorqueKpKd(
            zero_position, zero_velocity, torque, zero_kp, zero_kd);
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

        // 必须与 dual_arm_impedance_calc.cpp 完全一致。
        const std::vector<double> left_gravity_gain =
            {0.85, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0};
        const std::vector<double> right_gravity_gain =
            {1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0};
        const std::vector<double> kp =
            {4.0, 8.0, 8.0, 3.0, 2.0, 1.0, 0.8};
        const std::vector<double> kd =
            {0.6, 0.8, 0.8, 0.4, 0.25, 0.15, 0.1};
        const std::vector<double> torque_limit =
            {15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0};

        std::cout << "\n" << std::string(72, '=') << std::endl;
        std::cout << "双臂关节空间阻抗控制" << std::endl;
        std::cout << "tau = G(q) + Kp(q_target - q) - Kd*dq" << std::endl;
        std::cout << std::string(72, '=') << std::endl;
        std::cout << "左臂端口: /dev/ttyACM0" << std::endl;
        std::cout << "右臂端口: /dev/ttyACM1" << std::endl;

        ArmContext left(
            "left", left_config, left_urdf,
            left_gravity_gain, kp, kd, torque_limit);
        ArmContext right(
            "right", right_config, right_urdf,
            right_gravity_gain, kp, kd, torque_limit);

        std::vector<double> left_target;
        std::vector<double> right_target;
        const bool has_left_target =
            parseTargetArgument(argc, argv, "--left-target", left_target);
        const bool has_right_target =
            parseTargetArgument(argc, argv, "--right-target", right_target);

        if (has_left_target) {
            auto position = left.readPosition();
            auto velocity = left.readVelocity();
            if (!validState(position, velocity, left.n)) {
                throw std::runtime_error("初始化后无法读取有效的左臂关节状态");
            }
            left.latchTarget(left_target);
        } else {
            left.latchCurrentTarget();
        }
        if (has_right_target) {
            auto position = right.readPosition();
            auto velocity = right.readVelocity();
            if (!validState(position, velocity, right.n)) {
                throw std::runtime_error("初始化后无法读取有效的右臂关节状态");
            }
            right.latchTarget(right_target);
        } else {
            right.latchCurrentTarget();
        }

        printVector("[left] q_target(rad): ", left.q_target);
        printVector("[right] q_target(rad):", right.q_target);
        std::cout << "\n开始双臂阻抗控制，按 Ctrl+C 退出...\n" << std::endl;

        auto next_tick = std::chrono::steady_clock::now();
        const auto loop_period = std::chrono::milliseconds(5);  // 200 Hz
        auto last_print = std::chrono::steady_clock::now();

        while (keep_running) {
            next_tick += loop_period;

            auto left_position = left.readPosition();
            auto left_velocity = left.readVelocity();
            auto right_position = right.readPosition();
            auto right_velocity = right.readVelocity();

            if (!validState(left_position, left_velocity, left.n) ||
                !validState(right_position, right_velocity, right.n)) {
                std::cerr << "[安全停止] 读取到无效的双臂关节状态，不再发送阻抗力矩。"
                          << std::endl;
                break;
            }

            auto left_torque = left.computeCommand(left_position, left_velocity);
            auto right_torque = right.computeCommand(right_position, right_velocity);
            left.sendCommand(left_torque);
            right.sendCommand(right_torque);

            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_print).count() >= 0.5) {
                printVector("[left] tau(Nm): ", left_torque);
                printVector("[right] tau(Nm):", right_torque);
                last_print = now;
            }

            std::this_thread::sleep_until(next_tick);
            if (std::chrono::steady_clock::now() > next_tick + loop_period) {
                next_tick = std::chrono::steady_clock::now();
            }
        }

        std::cout << "\n双臂阻抗控制已停止，电机将掉电。" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
