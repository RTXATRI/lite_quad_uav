/**
 * @file pid_position_controller.cpp
 * @brief PidAxis 与 PidPositionController 实现。
 *
 * 数学逻辑参考 src/3rd/pid.cpp：反馈量微分、条件积分 anti-windup、输出限幅。
 */

#include <lite_quad_uav/controllers/pid_position_controller.hpp>

#include <algorithm>
#include <cmath>

/* PidAxis ------------------------------------------------------------------*/

/**
 * @brief 构造函数，重置所有内部状态。
 */
PidAxis::PidAxis() {
    reset();
}

/**
 * @brief 设置 PID 增益。
 * @param[in] kp 比例增益。
 * @param[in] ki 积分增益。
 * @param[in] kd 微分增益。
 */
void PidAxis::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/**
 * @brief 设置输出和积分限幅。
 * @param[in] output_limit 输出限幅，0=不限。
 * @param[in] integral_limit 积分限幅，0=不限。
 */
void PidAxis::setLimits(double output_limit, double integral_limit) {
    output_limit_ = output_limit;
    integral_limit_ = integral_limit;
}

/**
 * @brief 设置误差死区，小于死区的误差置零（P/I/D 均不作用）。
 * @param[in] deadband 死区阈值。
 */
void PidAxis::setDeadband(double deadband) {
    deadband_ = deadband;
}

/**
 * @brief 设置最小 dt，防止除零。
 * @param[in] dt_min 最小时间步长 (s)，若 <=0 则强制为 0.001。
 */
void PidAxis::setDtMin(double dt_min) {
    dt_min_ = dt_min > 0.0 ? dt_min : 0.001;
}

/**
 * @brief 重置积分、微分和输出状态。
 */
void PidAxis::reset() {
    integral_ = 0.0;
    last_feedback_ = 0.0;
    last_output_ = 0.0;
    feedback_received_ = false;
}

/**
 * @brief 仅重置积分累积。
 */
void PidAxis::resetIntegral() {
    integral_ = 0.0;
}

/**
 * @brief 仅重置微分状态。
 */
void PidAxis::resetDerivative() {
    last_feedback_ = 0.0;
    feedback_received_ = false;
}

/**
 * @brief 执行单轴 PID 计算（线性版本）。
 * @param[in] setpoint 目标值。
 * @param[in] feedback 反馈值。
 * @param[in] dt 时间步长 (s)。
 * @return PID 输出。
 */
double PidAxis::update(double setpoint, double feedback, double dt) {
    return updateWithError(setpoint - feedback, feedback, dt, false);
}

/**
 * @brief 角度版 PID 计算，自动处理角度环绕。
 * @param[in] setpoint 目标角度 (rad)。
 * @param[in] feedback 反馈角度 (rad)。
 * @param[in] dt 时间步长 (s)。
 * @return PID 输出。
 */
double PidAxis::updateAngle(double setpoint, double feedback, double dt) {
    return updateWithError(normalizeAngle(setpoint - feedback), feedback, dt, true);
}

/**
 * @brief 获取当前积分累积值。
 * @return 积分值。
 */
double PidAxis::integral() const {
    return integral_;
}

/**
 * @brief 获取上一次输出值。
 * @return 上一次输出。
 */
double PidAxis::lastOutput() const {
    return last_output_;
}

/**
 * @brief 内部 PID 计算核心：误差计算、反馈微分、条件积分、输出限幅。
 * @param[in] error 已计算的误差值（角度已归一化）。
 * @param[in] feedback 反馈值（用于微分计算）。
 * @param[in] dt 时间步长。
 * @param[in] angular_feedback 是否为角度反馈（影响微分差值归一化）。
 * @return PID 输出。
 *
 * @note 微分项 = -delta_feedback / dt（非 delta_error / dt），避免目标阶跃时产生微分冲击。
 * @note 条件积分：integral_in_limit && output_not_saturated 同时满足才累积积分。
 */
double PidAxis::updateWithError(double error, double feedback, double dt, bool angular_feedback) {
    // 非法输入保护
    if (!std::isfinite(error) || !std::isfinite(feedback) || !std::isfinite(dt) || dt <= 0.0) {
        return last_output_;
    }

    // 死区处理：误差在死区内置零
    if (std::fabs(error) < deadband_) {
        error = 0.0;
    }

    // 反馈量微分（derivative on measurement）
    double derivative = 0.0;
    if (feedback_received_) {
        double feedback_delta = angular_feedback ? normalizeAngle(feedback - last_feedback_) : (feedback - last_feedback_);
        derivative = -feedback_delta / std::max(dt, dt_min_);
    }

    if (!std::isfinite(derivative)) {
        derivative = 0.0;
    }

    // PD 输出
    double output_pd = kp_ * error + kd_ * derivative;

    // 条件积分（anti-windup）
    if (ki_ > 0.0) {
        double integral_new = integral_ + error * dt;
        if (std::isfinite(integral_new)) {
            bool integral_in_limit = integral_limit_ <= 0.0 || std::fabs(integral_new) <= integral_limit_;
            bool output_not_saturated = output_limit_ <= 0.0 || std::fabs(output_pd + integral_new * ki_) <= output_limit_;
            if (integral_in_limit && output_not_saturated) {
                integral_ = integral_new;
            }
        }
    }

    // 最终输出
    double output = output_pd + integral_ * ki_;
    if (std::isfinite(output)) {
        if (output_limit_ > 0.0) {
            output = clamp(output, -output_limit_, output_limit_);
        }
        last_output_ = output;
    }

    last_feedback_ = feedback;
    feedback_received_ = true;
    return last_output_;
}

/**
 * @brief 数值限幅。
 * @param[in] val 待限幅值。
 * @param[in] lo 下限。
 * @param[in] hi 上限。
 * @return 限幅后值。
 */
double PidAxis::clamp(double val, double lo, double hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief 角度归一化到 [-π, π)。
 * @param[in] angle 输入角度 (rad)。
 * @return 归一化后角度。
 */
double PidAxis::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/* PidPositionController ----------------------------------------------------*/

/**
 * @brief 构造函数。
 */
PidPositionController::PidPositionController() {
}

/**
 * @brief 从 ROS 参数服务器读取各轴 PID 参数并配置。
 * @param[in] nh PID 参数命名空间句柄。
 */
void PidPositionController::loadParams(ros::NodeHandle& nh) {
    double kp_xy = nh.param<double>("kp_xy", 0.6);
    double ki_xy = nh.param<double>("ki_xy", 0.0);
    double kd_xy = nh.param<double>("kd_xy", 0.05);

    double kp_z = nh.param<double>("kp_z", 0.8);
    double ki_z = nh.param<double>("ki_z", 0.0);
    double kd_z = nh.param<double>("kd_z", 0.05);

    double kp_yaw = nh.param<double>("kp_yaw", 0.5);
    double ki_yaw = nh.param<double>("ki_yaw", 0.0);
    double kd_yaw = nh.param<double>("kd_yaw", 0.0);

    max_xy_speed_ = nh.param<double>("max_xy_speed", 1.0);
    max_z_up_speed_ = nh.param<double>("max_z_up_speed", 0.6);
    max_z_down_speed_ = nh.param<double>("max_z_down_speed", 0.6);
    max_yaw_rate_ = nh.param<double>("max_yaw_rate", 0.5);

    double integral_limit_xy = nh.param<double>("integral_limit_xy", 0.5);
    double integral_limit_z = nh.param<double>("integral_limit_z", 0.3);
    double integral_limit_yaw = nh.param<double>("integral_limit_yaw", 0.2);

    double deadband_xy = nh.param<double>("deadband_xy", 0.02);
    double deadband_z = nh.param<double>("deadband_z", 0.02);
    double deadband_yaw = nh.param<double>("deadband_yaw", 0.01);
    double dt_min = nh.param<double>("dt_min", 0.001);

    converge_radius_ = nh.param<double>("converge_radius", 0.4);

    pid_x_.setGains(kp_xy, ki_xy, kd_xy);
    pid_y_.setGains(kp_xy, ki_xy, kd_xy);
    pid_z_.setGains(kp_z, ki_z, kd_z);
    pid_yaw_.setGains(kp_yaw, ki_yaw, kd_yaw);

    double z_axis_pre_limit = std::max(max_z_up_speed_, max_z_down_speed_);
    pid_x_.setLimits(max_xy_speed_, integral_limit_xy);
    pid_y_.setLimits(max_xy_speed_, integral_limit_xy);
    pid_z_.setLimits(z_axis_pre_limit, integral_limit_z);
    pid_yaw_.setLimits(max_yaw_rate_, integral_limit_yaw);

    pid_x_.setDeadband(deadband_xy);
    pid_y_.setDeadband(deadband_xy);
    pid_z_.setDeadband(deadband_z);
    pid_yaw_.setDeadband(deadband_yaw);

    pid_x_.setDtMin(dt_min);
    pid_y_.setDtMin(dt_min);
    pid_z_.setDtMin(dt_min);
    pid_yaw_.setDtMin(dt_min);
}

/**
 * @brief 重置所有轴 PID 状态。
 */
void PidPositionController::reset() {
    pid_x_.reset();
    pid_y_.reset();
    pid_z_.reset();
    pid_yaw_.reset();
}

/**
 * @brief 执行四轴位置 PID 计算，输出带限幅的速度。
 * @param[in] vehicle 飞行器当前状态。
 * @param[in] command 控制命令（含目标位置）。
 * @param[in] dt 时间步长 (s)。
 * @return 控制器输出。
 *
 * @note 积分采用条件 anti-windup：仅当积分值未超限且总输出未饱和时才累积，防止积分过冲。
 * @note XY 速度采用矢量限幅（保持方向不变缩放），Z 和 YAW 采用标量限幅。
 * @note 若 command.use_yaw 为 false，pid_yaw_ 会被重置（含微分状态）。
 */
ControllerOutput PidPositionController::update(const VehicleState& vehicle, const ControllerCommand& command, double dt) {
    ControllerOutput output;
    output.controller_name = "pid_position";

    // 输入有效性检查
    if (!vehicle.valid || !command.valid) {
        output.valid = false;
        output.message = vehicle.valid ? "Command invalid" : "Vehicle state invalid";
        return output;
    }

    // 各轴 PID 计算
    double vx = pid_x_.update(command.target_x, vehicle.x, dt);
    double vy = pid_y_.update(command.target_y, vehicle.y, dt);
    double vz = pid_z_.update(command.target_z, vehicle.z, dt);

    // 偏航控制：use_yaw=false 时重置 PID（含微分状态）
    double yaw_rate = 0.0;
    if (command.use_yaw) {
        yaw_rate = pid_yaw_.updateAngle(command.target_yaw, vehicle.yaw, dt);
    } else {
        pid_yaw_.reset();
    }

    // 速度限幅：命令级字段为真实限幅，0 表示该方向不允许运动。
    double max_xy_speed = command.max_xy_speed;
    double max_z_up_speed = command.max_z_up_speed;
    double max_z_down_speed = command.max_z_down_speed;
    double max_yaw_rate = command.max_yaw_rate;

    // XY 矢量限幅（保持方向不变）
    bool xy_saturated = false;
    double speed_xy = std::sqrt(vx * vx + vy * vy);
    if (speed_xy > max_xy_speed && speed_xy > 0.0) {
        double scale = max_xy_speed / speed_xy;
        vx *= scale;
        vy *= scale;
        xy_saturated = true;
    }

    // Z 按上下方向分别限幅
    bool z_saturated = false;
    if (vz > max_z_up_speed) {
        vz = max_z_up_speed;
        z_saturated = true;
    } else if (vz < -max_z_down_speed) {
        vz = -max_z_down_speed;
        z_saturated = true;
    }

    // YAW 标量限幅
    bool yaw_saturated = false;
    if (yaw_rate > max_yaw_rate) {
        yaw_rate = max_yaw_rate;
        yaw_saturated = true;
    } else if (yaw_rate < -max_yaw_rate) {
        yaw_rate = -max_yaw_rate;
        yaw_saturated = true;
    }

    // 计算位置误差和收敛状态
    double ex = command.target_x - vehicle.x;
    double ey = command.target_y - vehicle.y;
    double ez = command.target_z - vehicle.z;
    double pos_err = std::sqrt(ex * ex + ey * ey + ez * ez);

    output.vx = vx;
    output.vy = vy;
    output.vz = vz;
    output.yaw_rate = yaw_rate;
    output.valid = true;
    output.position_error_norm = pos_err;
    output.target_distance = pos_err;
    output.converged = pos_err < converge_radius_;
    output.saturated = xy_saturated || z_saturated || yaw_saturated;

    return output;
}

/**
 * @brief 角度归一化到 [-π, π)。
 * @param[in] angle 输入角度 (rad)。
 * @return 归一化后角度。
 */
double PidPositionController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
