/**
 * @file left_arm_teleop_force_feedback_dob.cpp
 * @brief 左臂主从遥操 - 七轴广义动量 DOB 力反馈 + 夹爪直接力矩映射
 *
 * 主臂: /dev/ttyACM0 (litearm_left_arm.yaml, serial_id=1)
 * 从臂: /dev/ttyACM1 (litearm_left_arm_follower.yaml, serial_id=2)
 *
 * 工作原理（参考 Panthera 5_teleop_control_force_feedback_dob.py）：
 *   1. 主臂运行重力补偿 + 摩擦前馈，可被自由拖动
 *   2. 从臂用 MIT 模式 (kp/kd + 重力前馈) 跟踪主臂关节角度
 *   3. 从臂侧广义动量扰动观测器 (DOB) 估计环境外力矩：
 *        p = M(q) v
 *        p_dot = tau_motor - (G + tau_f - C^T v) + tau_ext
 *        r = Kobs * (p - ∫(p_dot_nominal + r) dt)  ->  tau_ext 估计
 *   4. 估计外力矩经 增益 -> 限幅 -> 斜率限制 后叠加到主臂，形成力反馈
 *   5. 夹爪不用 DOB：从臂夹爪实测力矩直接反向映射到主臂夹爪
 *
 * 启动流程: 观测器预热 -> 从臂同步 -> 静态偏置校准 -> 力反馈渐入
 * 启动期间请保持从臂无接触；屏幕显示“力反馈 100%”后再操作。
 *
 * 使用方法：
 *   ros2 run litearm_robot left_arm_teleop_force_feedback_dob
 *   或指定参数：
 *   ros2 run litearm_robot left_arm_teleop_force_feedback_dob \
 *       <leader.yaml> <follower.yaml> <left_arm.urdf> <feedback_gain> <gripper_gain>
 */

#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <cmath>
#include <algorithm>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace pin = pinocchio;

volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        keep_running = 0;
        std::cout << "\n\n收到停止请求，主从臂即将停机，请注意安全！" << std::endl;
    }
}

// ==================== 常量参数（按实机调试） ====================

static Eigen::VectorXd makeVec(std::initializer_list<double> v)
{
    Eigen::VectorXd out((int)v.size());
    int i = 0;
    for (double x : v) out[i++] = x;
    return out;
}

// 从臂位置跟踪增益 (MIT 模式 kp/kd)
// motor type: J1/J2 7256_35, J3/J4 6056_36, J5 4438_30, J6 5047_36, J7 4438_30
const Eigen::VectorXd FOLLOWER_KP = makeVec({20.0, 25.0, 25.0, 15.0, 5.0, 8.0, 4.0});
const Eigen::VectorXd FOLLOWER_KD = makeVec({1.5, 2.0, 2.0, 1.0, 0.3, 0.5, 0.2});

// 摩擦模型（库仑 + 粘滞），未标定，宁小勿大；标定后可加大以改善 DOB 精度
const Eigen::VectorXd COULOMB_FRICTION = makeVec({0.10, 0.10, 0.10, 0.08, 0.04, 0.05, 0.03});
const Eigen::VectorXd VISCOUS_FRICTION = makeVec({0.05, 0.05, 0.05, 0.03, 0.02, 0.02, 0.01});
const double FRICTION_SMOOTHING_VELOCITY = 0.05;   // tanh 平滑速度 (rad/s)

// 力矩安全限幅（与 left_arm_gravity_compensation 一致）
const Eigen::VectorXd TOTAL_TORQUE_LIMIT = makeVec({15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0});
// 主臂力反馈限幅 / 斜率限制 (Nm, Nm/s)
// 限幅只做异常保护，不应在正常接触中饱和，否则主臂感受到的力比实际偏小
const Eigen::VectorXd FEEDBACK_TORQUE_LIMIT = makeVec({8.0, 12.0, 12.0, 8.0, 3.0, 3.0, 2.0});
const Eigen::VectorXd FEEDBACK_TORQUE_RATE_LIMIT = makeVec({80.0, 120.0, 120.0, 80.0, 40.0, 40.0, 30.0});
// DOB 输出超过该值视为发散，重置观测器
// 必须大于正常接触可达的力矩，否则用力推从臂会触发重置、力反馈瞬间消失
const Eigen::VectorXd DOB_RESET_LIMIT = makeVec({30.0, 50.0, 50.0, 30.0, 12.0, 12.0, 8.0});
// 主臂阻尼，抑制力反馈引起的振荡
const Eigen::VectorXd MASTER_DAMPING = makeVec({0.12, 0.20, 0.20, 0.08, 0.04, 0.04, 0.03});

// 重力补偿力矩增益系数（与 left_arm_gravity_compensation 一致，主从臂共用）
const Eigen::VectorXd GRAVITY_GAIN = makeVec({0.85, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0});

// 从臂跟踪误差饱和 (rad)：目标位置不超过当前位置 ±该值，
// 限制 kp*(target-pos) 的最大堵转力矩（如 J2: 25*0.3=7.5Nm），
// 防止从臂被卡住/主臂快速甩动时电机持续大力矩堵转触发过流过热保护。
// 注意：这不限制从臂可到达的位置——从臂会持续向主臂位置移动直至完全到位
const Eigen::VectorXd MAX_TRACKING_ERROR = makeVec({0.3, 0.3, 0.3, 0.3, 0.4, 0.4, 0.4});

// ---------- 夹爪（第 8 个电机, 4438_30）：位置直接映射 + 力矩直接反馈，不用 DOB ----------
const double GRIPPER_KP = 6.0;
const double GRIPPER_KD = 0.3;
const double GRIPPER_FRICTION = 0.05;
const double GRIPPER_FRICTION_SMOOTHING_VELOCITY = 0.10;
const double GRIPPER_FEEDBACK_GAIN = 1.0;           // 夹爪力矩映射增益（独立于手臂增益，argv[5] 可调）
const double GRIPPER_DIRECT_FEEDBACK_LIMIT = 2.0;   // 反馈力矩限幅 (Nm)
const double GRIPPER_TOTAL_TORQUE_LIMIT = 2.5;      // 主臂夹爪总力矩限幅 (Nm)
const double GRIPPER_DIRECT_FEEDBACK_RATE_LIMIT = 30.0;  // Nm/s
const double GRIPPER_FEEDBACK_SIGN = -1.0;          // 方向不对时改成 +1.0
const double GRIPPER_POS_LOWER = -3.0;              // 夹爪目标安全限幅 (rad)，不截断正常行程
const double GRIPPER_POS_UPPER = 3.0;
// 夹爪跟踪误差饱和 (rad)：主爪拉超从爪机械行程时，限制堵转力矩 kp*0.4=2.4Nm，
// 防止 4438 电机堵转过流保护掉线
const double GRIPPER_MAX_TRACKING_ERROR = 0.4;

// ---------- 频率 / 时序 ----------
const double CONTROL_RATE_HZ = 200.0;               // 与重力补偿例程一致
const double PRINT_RATE_HZ = 2.0;
const double DOB_CUTOFF_HZ = 3.0;                   // DOB 带宽
const double VELOCITY_CUTOFF_HZ = 12.0;             // 观测器速度低通
const double OBSERVER_WARMUP_TIME = 1.0;            // 观测器预热 (s)
const double SYNC_TIME = 3.0;                       // 从臂同步到主臂位置 (s)
const double BIAS_TIME = 1.0;                       // 静态偏置校准 (s)
const double FEEDBACK_RAMP_TIME = 1.0;              // 力反馈渐入 (s)
const double DEFAULT_FEEDBACK_GAIN = 1.0;           // 力反馈增益，1.0 = 外力矩 1:1 映射到主臂
                                                    // 自由移动时拖拽感偏重可降低（argv[4]）

// ==================== 工具函数 ====================

Eigen::VectorXd toEigen(const std::vector<double>& v)
{
    return Eigen::Map<const Eigen::VectorXd>(v.data(), (int)v.size());
}

std::vector<double> toStd(const Eigen::VectorXd& v)
{
    return std::vector<double>(v.data(), v.data() + v.size());
}

Eigen::VectorXd clampVec(const Eigen::VectorXd& v, const Eigen::VectorXd& limit)
{
    return v.cwiseMax(-limit).cwiseMin(limit);
}

// 斜率限制：每周期变化量不超过 max_rate * dt
Eigen::VectorXd rateLimit(const Eigen::VectorXd& target,
                          const Eigen::VectorXd& previous,
                          const Eigen::VectorXd& max_rate,
                          double dt)
{
    Eigen::VectorXd max_step = max_rate * dt;
    Eigen::VectorXd delta = (target - previous).cwiseMax(-max_step).cwiseMin(max_step);
    return previous + delta;
}

double clampScalar(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

// 库仑 + 粘滞摩擦前馈
Eigen::VectorXd frictionTorque(const Eigen::VectorXd& velocity)
{
    Eigen::VectorXd tanh_v = (velocity / FRICTION_SMOOTHING_VELOCITY).array().tanh();
    return COULOMB_FRICTION.cwiseProduct(tanh_v) + VISCOUS_FRICTION.cwiseProduct(velocity);
}

double gripperFriction(double velocity)
{
    return GRIPPER_FRICTION * std::tanh(velocity / GRIPPER_FRICTION_SMOOTHING_VELOCITY);
}

void checkFinite(const std::string& name, const Eigen::VectorXd& v)
{
    if (!v.allFinite()) {
        throw std::runtime_error(name + " 包含 NaN 或 Inf，停止控制");
    }
}

void checkFinite(const std::string& name, double v)
{
    if (!std::isfinite(v)) {
        throw std::runtime_error(name + " 包含 NaN 或 Inf，停止控制");
    }
}

std::string vecToStr(const Eigen::VectorXd& v, int precision = 2)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "[";
    for (int i = 0; i < v.size(); ++i) {
        oss << std::setw(6) << v[i] << (i < v.size() - 1 ? " " : "");
    }
    oss << "]";
    return oss.str();
}

// ==================== 滤波器 / 观测器 ====================

// 一阶低通滤波器
class FirstOrderLowPass
{
public:
    explicit FirstOrderLowPass(double cutoff_hz) : cutoff_hz_(cutoff_hz), initialized_(false) {}

    Eigen::VectorXd update(const Eigen::VectorXd& sample, double dt)
    {
        if (!initialized_) {
            state_ = sample;
            initialized_ = true;
            return state_;
        }
        double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff_hz_ * dt);
        state_ += alpha * (sample - state_);
        return state_;
    }

    void reset() { initialized_ = false; }

private:
    double cutoff_hz_;
    bool initialized_;
    Eigen::VectorXd state_;
};

// 广义动量扰动观测器：输出环境施加到从臂的关节外力矩估计
class MomentumDisturbanceObserver
{
public:
    explicit MomentumDisturbanceObserver(double cutoff_hz)
        : gain_(2.0 * M_PI * cutoff_hz), initialized_(false) {}

    void reset() { initialized_ = false; }

    Eigen::VectorXd update(const Eigen::VectorXd& momentum,
                           const Eigen::VectorXd& nominal_momentum_rate,
                           double dt)
    {
        if (!initialized_) {
            integral_ = momentum;
            residual_ = Eigen::VectorXd::Zero(momentum.size());
            initialized_ = true;
            return residual_;
        }
        integral_ += dt * (nominal_momentum_rate + residual_);
        residual_ = gain_ * (momentum - integral_);
        return residual_;
    }

private:
    double gain_;
    bool initialized_;
    Eigen::VectorXd integral_;
    Eigen::VectorXd residual_;
};

// 静态偏置在线均值（校准阶段扣除 DOB 稳态偏置）
class RunningBias
{
public:
    explicit RunningBias(int size) : value_(Eigen::VectorXd::Zero(size)), samples_(0) {}

    void reset()
    {
        value_.setZero();
        samples_ = 0;
    }

    void update(const Eigen::VectorXd& sample)
    {
        samples_ += 1;
        value_ += (sample - value_) / (double)samples_;
    }

    const Eigen::VectorXd& value() const { return value_; }

private:
    Eigen::VectorXd value_;
    int samples_;
};

// ==================== 阶段机 ====================

struct PhaseState
{
    std::string name;
    double sync_activation;       // 0->1 从臂同步进度
    double feedback_activation;   // 0->1 力反馈渐入进度
};

PhaseState getPhase(double elapsed)
{
    const double warmup_end = OBSERVER_WARMUP_TIME;
    const double sync_end = warmup_end + SYNC_TIME;
    const double bias_end = sync_end + BIAS_TIME;

    if (elapsed < warmup_end) {
        return {"观测器预热", 0.0, 0.0};
    }
    if (elapsed < sync_end) {
        return {"从臂同步", (elapsed - warmup_end) / SYNC_TIME, 0.0};
    }
    if (elapsed < bias_end) {
        return {"静态校准", 1.0, 0.0};
    }
    double activation = std::min((elapsed - bias_end) / FEEDBACK_RAMP_TIME, 1.0);
    return {"力反馈", 1.0, activation};
}

// ==================== 动力学计算 ====================

// 重力力矩 G(q)，按关节应用重力增益
Eigen::VectorXd computeGravity(pin::Model& model, pin::Data& data, const Eigen::VectorXd& q)
{
    pin::computeGeneralizedGravity(model, data, q);
    return GRAVITY_GAIN.cwiseProduct(data.g);
}

// ==================== 主程序 ====================

int main(int argc, char** argv)
{
    try {
        signal(SIGINT, signal_handler);

        // 主臂配置 (serial_id=1 -> /dev/ttyACM0)
        std::string leader_config = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_left_arm.yaml";
        if (argc > 1) leader_config = argv[1];

        // 从臂配置 (serial_id=2 -> /dev/ttyACM1)
        std::string follower_config = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/litearm_left_arm_follower.yaml";
        if (argc > 2) follower_config = argv[2];

        // URDF 路径（仅左臂）
        std::string urdf_path = ament_index_cpp::get_package_share_directory("litearm_robot")
            + "/urdf/LiteArm_A10_251224_left_arm.urdf";
        if (argc > 3) urdf_path = argv[3];

        double feedback_gain = DEFAULT_FEEDBACK_GAIN;
        if (argc > 4) feedback_gain = std::stod(argv[4]);
        if (feedback_gain < 0.0 || feedback_gain > 1.5) {
            std::cerr << "错误: feedback_gain 必须在 [0, 1.5] 内" << std::endl;
            return 1;
        }

        double gripper_feedback_gain = GRIPPER_FEEDBACK_GAIN;
        if (argc > 5) gripper_feedback_gain = std::stod(argv[5]);
        if (gripper_feedback_gain < 0.0 || gripper_feedback_gain > 2.0) {
            std::cerr << "错误: gripper_feedback_gain 必须在 [0, 2] 内" << std::endl;
            return 1;
        }

        std::cout << "主臂配置: " << leader_config << std::endl;
        std::cout << "从臂配置: " << follower_config << std::endl;
        std::cout << "URDF文件: " << urdf_path << std::endl;

        // 初始化主从臂
        std::cout << "\n初始化主臂 (/dev/ttyACM0)..." << std::endl;
        litearm_robot::LiteArm leader(leader_config);
        std::cout << "\n初始化从臂 (/dev/ttyACM1)..." << std::endl;
        litearm_robot::LiteArm follower(follower_config);

        int n = leader.getMotorCount();
        if (n != follower.getMotorCount()) {
            std::cerr << "错误: 主臂电机数 " << n << " != 从臂电机数 "
                      << follower.getMotorCount() << std::endl;
            return 1;
        }
        if ((int)FOLLOWER_KP.size() != n) {
            std::cerr << "错误: 该程序仅配置为 " << FOLLOWER_KP.size()
                      << " 自由度，实际电机数 " << n << std::endl;
            return 1;
        }
        std::cout << "手臂电机数量: " << n << " (另含夹爪电机)" << std::endl;

        // 加载 Pinocchio 模型
        pin::Model model;
        pin::urdf::buildModel(urdf_path, model);
        pin::Data leader_data(model);
        pin::Data follower_data(model);
        std::cout << "Pinocchio 模型: nq=" << model.nq << ", nv=" << model.nv << std::endl;
        if (model.nq != n) {
            std::cerr << "错误: Pinocchio nq=" << model.nq << " != 电机数量 " << n
                      << ", 请检查URDF文件" << std::endl;
            return 1;
        }

        // 注：从臂目标直接映射主臂位置，不做关节限位夹紧——
        // 主从为同构左臂，主臂物理可达的位置从臂同样可达

        // 观测器 / 滤波器 / 偏置
        MomentumDisturbanceObserver observer(DOB_CUTOFF_HZ);
        FirstOrderLowPass velocity_filter(VELOCITY_CUTOFF_HZ);
        RunningBias disturbance_bias(n);
        Eigen::VectorXd previous_feedback = Eigen::VectorXd::Zero(n);
        double previous_gripper_feedback = 0.0;
        double gripper_torque_bias = 0.0;    // 从臂夹爪力矩静态偏置（校准阶段测得）
        int gripper_bias_samples = 0;
        bool post_sync_initialized = false;
        int observer_reset_count = 0;

        std::vector<double> zero_vec(n, 0.0);
        std::vector<double> kp_std = toStd(FOLLOWER_KP);
        std::vector<double> kd_std = toStd(FOLLOWER_KD);

        // 先读取当前位置，确认通信正常
        leader.send_get_motor_state_cmd();
        leader.motor_send_cmd();
        follower.send_get_motor_state_cmd();
        follower.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        Eigen::VectorXd follower_start_pos = toEigen(follower.getCurrentPos());
        double follower_start_gripper = follower.getCurrentPosGripper();
        Eigen::VectorXd leader_init_pos = toEigen(leader.getCurrentPos());

        std::cout << "主臂初始位置(rad): " << vecToStr(leader_init_pos, 3) << std::endl;
        std::cout << "从臂初始位置(rad): " << vecToStr(follower_start_pos, 3) << std::endl;

        std::cout << "\n" << std::string(78, '=') << std::endl;
        std::cout << "七轴 DOB 力反馈遥操 + 夹爪直接力矩映射已启动" << std::endl;
        std::cout << "DOB/速度滤波: " << DOB_CUTOFF_HZ << "/" << VELOCITY_CUTOFF_HZ << " Hz"
                  << "  力反馈增益: " << feedback_gain << std::endl;
        std::cout << "反馈链路: 从臂DOB外力矩 -> 增益 -> 限幅/斜率 -> 主臂" << std::endl;
        std::cout << "夹爪链路: " << (GRIPPER_FEEDBACK_SIGN < 0 ? "-" : "+")
                  << "(从臂夹爪实测力矩-偏置) x 增益" << gripper_feedback_gain
                  << " -> 限幅/斜率 -> 主臂夹爪" << std::endl;
        std::cout << "流程: 预热(" << OBSERVER_WARMUP_TIME << "s) -> 同步(" << SYNC_TIME
                  << "s) -> 静态校准(" << BIAS_TIME << "s) -> 力反馈渐入("
                  << FEEDBACK_RAMP_TIME << "s)" << std::endl;
        std::cout << "注意: 启动期间请保持双臂无接触；显示“力反馈 100%”后再操作。" << std::endl;
        std::cout << "按 Ctrl+C 停止，停止后电机掉电，请注意安全！" << std::endl;
        std::cout << std::string(78, '=') << "\n" << std::endl;

        const double period = 1.0 / CONTROL_RATE_HZ;
        const double print_period = 1.0 / PRINT_RATE_HZ;
        const double sync_end = OBSERVER_WARMUP_TIME + SYNC_TIME;
        const double bias_end = sync_end + BIAS_TIME;

        auto start_time = std::chrono::steady_clock::now();
        auto last_time = start_time;
        auto next_tick = start_time;
        auto next_print = start_time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(print_period));
        int report_cycles = 0;
        auto report_start = start_time;

        while (keep_running) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            double dt = clampScalar(
                std::chrono::duration<double>(now - last_time).count(), 0.0002, 0.02);
            last_time = now;

            PhaseState phase = getPhase(elapsed);

            // 1. 请求并读取主从臂状态
            leader.send_get_motor_state_cmd();
            leader.motor_send_cmd();
            follower.send_get_motor_state_cmd();
            follower.motor_send_cmd();

            Eigen::VectorXd leader_pos = toEigen(leader.getCurrentPos());
            Eigen::VectorXd leader_vel = toEigen(leader.getCurrentVel());
            Eigen::VectorXd follower_pos = toEigen(follower.getCurrentPos());
            Eigen::VectorXd follower_vel = toEigen(follower.getCurrentVel());
            Eigen::VectorXd follower_torque = toEigen(follower.getCurrentTorque());
            double leader_gripper_pos = leader.getCurrentPosGripper();
            double leader_gripper_vel = leader.getCurrentVelGripper();
            double follower_gripper_torque = follower.getCurrentTorqueGripper();
            double follower_gripper_pos = follower.getCurrentPosGripper();

            checkFinite("leader_pos", leader_pos);
            checkFinite("leader_vel", leader_vel);
            checkFinite("follower_pos", follower_pos);
            checkFinite("follower_vel", follower_vel);
            checkFinite("follower_torque", follower_torque);
            checkFinite("leader_gripper_pos", leader_gripper_pos);
            checkFinite("leader_gripper_vel", leader_gripper_vel);
            checkFinite("follower_gripper_torque", follower_gripper_torque);

            // 2. 动力学前馈
            Eigen::VectorXd leader_gravity = computeGravity(model, leader_data, leader_pos);
            Eigen::VectorXd follower_gravity = computeGravity(model, follower_data, follower_pos);
            Eigen::VectorXd leader_friction = frictionTorque(leader_vel);
            Eigen::VectorXd follower_friction = frictionTorque(follower_vel);
            Eigen::VectorXd observer_velocity = velocity_filter.update(follower_vel, dt);
            Eigen::VectorXd observer_friction = frictionTorque(observer_velocity);

            // 3. 广义动量 DOB 估计从臂外力矩
            //    M q_ddot + C q_dot + G + tau_f = tau_motor + tau_ext
            //    p_dot = tau_motor - (G + tau_f - C^T q_dot) + tau_ext
            pin::crba(model, follower_data, follower_pos);
            follower_data.M.triangularView<Eigen::StrictlyLower>() =
                follower_data.M.transpose().triangularView<Eigen::StrictlyLower>();
            pin::computeCoriolisMatrix(model, follower_data, follower_pos, observer_velocity);

            Eigen::VectorXd momentum = follower_data.M * observer_velocity;
            Eigen::VectorXd beta = follower_gravity + observer_friction
                - follower_data.C.transpose() * observer_velocity;
            Eigen::VectorXd nominal_momentum_rate = follower_torque - beta;
            checkFinite("momentum", momentum);
            checkFinite("nominal_momentum_rate", nominal_momentum_rate);

            Eigen::VectorXd disturbance_raw =
                observer.update(momentum, nominal_momentum_rate, dt);

            // DOB 发散保护
            bool observer_reset_this_cycle = false;
            if ((disturbance_raw.cwiseAbs().array() > DOB_RESET_LIMIT.array()).any()) {
                observer.reset();
                previous_feedback.setZero();
                observer_reset_count++;
                observer_reset_this_cycle = true;
                disturbance_raw.setZero();
            }

            // 同步结束时重置观测器与偏置，从干净状态进入校准
            if (elapsed >= sync_end && !post_sync_initialized) {
                observer.reset();
                disturbance_bias.reset();
                previous_feedback.setZero();
                previous_gripper_feedback = 0.0;
                gripper_torque_bias = 0.0;
                gripper_bias_samples = 0;
                post_sync_initialized = true;
                disturbance_raw.setZero();
            }
            // 静态校准阶段累计偏置（手臂 DOB 偏置 + 夹爪力矩偏置）
            if (elapsed >= sync_end && elapsed < bias_end) {
                disturbance_bias.update(disturbance_raw);
                gripper_bias_samples++;
                gripper_torque_bias +=
                    (follower_gripper_torque - gripper_torque_bias) / gripper_bias_samples;
            }

            Eigen::VectorXd disturbance = disturbance_raw - disturbance_bias.value();
            if (observer_reset_this_cycle) {
                disturbance.setZero();
            }

            // 4. 力反馈：增益 -> 限幅 -> 斜率限制
            Eigen::VectorXd feedback_target = clampVec(
                phase.feedback_activation * feedback_gain * disturbance,
                FEEDBACK_TORQUE_LIMIT);
            Eigen::VectorXd feedback_torque = rateLimit(
                feedback_target, previous_feedback, FEEDBACK_TORQUE_RATE_LIMIT, dt);
            if (phase.feedback_activation <= 0.0 || observer_reset_this_cycle) {
                feedback_torque.setZero();
            }
            previous_feedback = feedback_torque;

            // 5. 主臂: 重力补偿 + 摩擦前馈 + 力反馈 - 阻尼（纯力矩模式）
            Eigen::VectorXd leader_torque_cmd = clampVec(
                leader_gravity + leader_friction + feedback_torque
                    - MASTER_DAMPING.cwiseProduct(leader_vel),
                TOTAL_TORQUE_LIMIT);

            // 6. 从臂: 位置跟踪 + 重力/摩擦前馈
            Eigen::VectorXd follower_torque_ff = clampVec(
                follower_gravity + follower_friction, TOTAL_TORQUE_LIMIT);
            Eigen::VectorXd follower_target_pos =
                (1.0 - phase.sync_activation) * follower_start_pos
                + phase.sync_activation * leader_pos;
            Eigen::VectorXd follower_target_vel = phase.sync_activation * leader_vel;

            // 跟踪误差饱和：目标不超出从臂当前位置 ±MAX_TRACKING_ERROR，
            // 限制电机内部 kp*(target-pos) 的最大力矩，防止堵转过流。
            // 不做关节限位夹紧，主臂位置直接映射到从臂
            follower_target_pos = follower_target_pos
                .cwiseMax(follower_pos - MAX_TRACKING_ERROR)
                .cwiseMin(follower_pos + MAX_TRACKING_ERROR);

            if (!leader.posVelTorqueKpKd(zero_vec, zero_vec, toStd(leader_torque_cmd),
                                         zero_vec, zero_vec)) {
                throw std::runtime_error("主臂控制指令被拒绝");
            }
            if (!follower.posVelTorqueKpKd(toStd(follower_target_pos),
                                           toStd(follower_target_vel),
                                           toStd(follower_torque_ff),
                                           kp_std, kd_std)) {
                throw std::runtime_error("从臂控制指令被拒绝");
            }

            // 7. 夹爪: 从臂实测力矩（扣除静态偏置）直接映射到主臂（不用 DOB）
            double gripper_feedback_target = clampScalar(
                phase.feedback_activation * gripper_feedback_gain * GRIPPER_FEEDBACK_SIGN
                    * (follower_gripper_torque - gripper_torque_bias),
                -GRIPPER_DIRECT_FEEDBACK_LIMIT, GRIPPER_DIRECT_FEEDBACK_LIMIT);
            double max_gripper_step = GRIPPER_DIRECT_FEEDBACK_RATE_LIMIT * dt;
            double gripper_feedback = previous_gripper_feedback + clampScalar(
                gripper_feedback_target - previous_gripper_feedback,
                -max_gripper_step, max_gripper_step);
            if (phase.feedback_activation <= 0.0) {
                gripper_feedback = 0.0;
            }
            previous_gripper_feedback = gripper_feedback;

            double leader_gripper_cmd = clampScalar(
                gripperFriction(leader_gripper_vel) + gripper_feedback,
                -GRIPPER_TOTAL_TORQUE_LIMIT, GRIPPER_TOTAL_TORQUE_LIMIT);
            double follower_gripper_target = clampScalar(
                (1.0 - phase.sync_activation) * follower_start_gripper
                    + phase.sync_activation * leader_gripper_pos,
                GRIPPER_POS_LOWER, GRIPPER_POS_UPPER);
            // 夹爪跟踪误差饱和：主爪拉超从爪机械行程时限制堵转力矩
            follower_gripper_target = clampScalar(
                follower_gripper_target,
                follower_gripper_pos - GRIPPER_MAX_TRACKING_ERROR,
                follower_gripper_pos + GRIPPER_MAX_TRACKING_ERROR);

            // 主臂夹爪: 纯力矩（kp=kd=0），从臂夹爪: 位置跟踪
            if (!leader.gripperControlMIT(leader_gripper_pos, 0.0,
                                          leader_gripper_cmd, 0.0, 0.0)) {
                throw std::runtime_error("主臂夹爪控制指令被拒绝");
            }
            if (!follower.gripperControlMIT(follower_gripper_target,
                                            phase.sync_activation * leader_gripper_vel,
                                            0.0, GRIPPER_KP, GRIPPER_KD)) {
                throw std::runtime_error("从臂夹爪控制指令被拒绝");
            }

            // 8. 定期打印状态
            report_cycles++;
            if (now >= next_print) {
                double actual_rate = report_cycles
                    / std::max(std::chrono::duration<double>(now - report_start).count(), 1e-6);
                int worst_joint = 0;
                double tracking_error =
                    (leader_pos - follower_pos).cwiseAbs().maxCoeff(&worst_joint);
                std::cout << std::fixed << std::setprecision(3)
                          << phase.name << " " << std::setw(3)
                          << (int)(phase.feedback_activation * 100) << "%"
                          << "  频率=" << std::setprecision(1) << actual_rate << "Hz"
                          << "  误差=J" << (worst_joint + 1) << ":"
                          << std::setprecision(3) << tracking_error << "rad"
                          << "  DOB=" << vecToStr(disturbance)
                          << "  反馈=" << vecToStr(feedback_torque)
                          << "  重置=" << observer_reset_count
                          << "  夹爪主/从=" << std::setprecision(2) << leader_gripper_pos
                          << "/" << follower_gripper_pos
                          << " 测量=" << (follower_gripper_torque - gripper_torque_bias)
                          << " 反馈=" << gripper_feedback << "Nm"
                          << std::endl;
                next_print = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(print_period));
                report_start = now;
                report_cycles = 0;
            }

            // 9. 固定周期
            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(period));
            auto now2 = std::chrono::steady_clock::now();
            if (next_tick > now2) {
                std::this_thread::sleep_for(next_tick - now2);
            } else {
                next_tick = now2;
            }
        }

        // 停机
        std::cout << "\n正在停机..." << std::endl;
        try { follower.set_stop(); } catch (...) { std::cerr << "警告: 从臂停机指令失败" << std::endl; }
        try { leader.set_stop(); } catch (...) { std::cerr << "警告: 主臂停机指令失败" << std::endl; }
        std::cout << "主臂和从臂已发送停机指令，电机掉电，请注意安全。" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
