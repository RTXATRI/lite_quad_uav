/**
 * @file pid_position_controller.hpp
 * @brief 基础 PID 位置控制器：X/Y/Z/YAW 目标转换为速度候选量。
 */

#pragma once

#include <ros/ros.h>

#include <lite_quad_uav/tasks/actions.hpp>
#include <lite_quad_uav/controllers/controller_command.hpp>
#include <lite_quad_uav/controllers/controller_types.hpp>

/* PidAxis: 单轴 PID ---------------------------------------------------------*/

/**
 * @brief 单轴 PID 控制器：支持输出限幅、积分限幅、死区和最小 dt 保护。
 */
class PidAxis {
public:
    PidAxis();

    /**
     * @brief 设置 PID 增益。
     * @param[in] kp 比例增益。
     * @param[in] ki 积分增益。
     * @param[in] kd 微分增益。
     */
    void setGains(double kp, double ki, double kd);

    /**
     * @brief 设置输出和积分限幅。
     * @param[in] output_limit 输出限幅，0=不限。
     * @param[in] integral_limit 积分限幅，0=不限。
     */
    void setLimits(double output_limit, double integral_limit);

    /**
     * @brief 设置误差死区，小于死区的误差置零（P/I/D 均不作用）。
     * @param[in] deadband 死区阈值。
     */
    void setDeadband(double deadband);

    /**
     * @brief 设置最小 dt，防止除零。
     * @param[in] dt_min 最小时间步长 (s)。
     */
    void setDtMin(double dt_min);

    /**
     * @brief 重置积分、微分和输出状态。
     */
    void reset();

    /**
     * @brief 仅重置积分累积。
     */
    void resetIntegral();

    /**
     * @brief 仅重置微分状态。
     */
    void resetDerivative();

    /**
     * @brief 执行单轴 PID 计算。
     * @param[in] setpoint 目标值。
     * @param[in] feedback 反馈值。
     * @param[in] dt 时间步长 (s)。
     * @return PID 输出。
     *
     * @note 微分项基于反馈量微分 (derivative on measurement)，非误差微分，
     *       避免目标突变时的微分冲击 (derivative kick)。
     */
    double update(double setpoint, double feedback, double dt);

    /**
     * @brief 角度版 PID 计算，自动处理角度环绕。
     * @param[in] setpoint 目标角度 (rad)。
     * @param[in] feedback 反馈角度 (rad)。
     * @param[in] dt 时间步长 (s)。
     * @return PID 输出。
     */
    double updateAngle(double setpoint, double feedback, double dt);

    /**
     * @brief 获取当前积分累积值。
     * @return 积分值。
     */
    double integral() const;

    /**
     * @brief 获取上一次输出值。
     * @return 上一次输出。
     */
    double lastOutput() const;

private:
    double kp_ = 0.0;              /**< 比例增益 */
    double ki_ = 0.0;              /**< 积分增益 */
    double kd_ = 0.0;              /**< 微分增益 */
    double integral_ = 0.0;        /**< 积分累积值 */
    double output_limit_ = 0.0;    /**< 输出限幅，0=不限 */
    double integral_limit_ = 0.0;  /**< 积分限幅，0=不限 */
    double deadband_ = 0.0;        /**< 误差死区阈值 */
    double dt_min_ = 0.001;        /**< 最小时间步长 (s) */
    double last_feedback_ = 0.0;   /**< 上一次反馈值，用于计算反馈微分 */
    double last_output_ = 0.0;     /**< 上一次输出值 */
    bool feedback_received_ = false; /**< 是否已收到至少一次反馈 */

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
    double updateWithError(double error, double feedback, double dt, bool angular_feedback);

    /**
     * @brief 数值限幅。
     * @param[in] val 待限幅值。
     * @param[in] lo 下限。
     * @param[in] hi 上限。
     * @return 限幅后值。
     */
    static double clamp(double val, double lo, double hi);

    /**
     * @brief 角度归一化到 [-π, π)。
     * @param[in] angle 输入角度 (rad)。
     * @return 归一化后角度。
     */
    static double normalizeAngle(double angle);
};

/* PidPositionController: 四轴 PID 位置控制器 --------------------------------*/

/**
 * @brief 四轴 PID 位置控制器：X/Y/Z 位置 + YAW 角度输入，输出速度候选量。
 */
class PidPositionController {
public:
    PidPositionController();

    /**
     * @brief 从 ROS 参数服务器加载各轴 PID 增益和限幅参数。
     * @param[in] nh PID 参数命名空间句柄。
     */
    void loadParams(ros::NodeHandle& nh);

    /**
     * @brief 重置所有轴 PID 状态。
     */
    void reset();

    /**
     * @brief 执行四轴位置 PID 计算，输出带限幅的速度。
     * @param[in] vehicle 飞行器当前状态。
     * @param[in] command 控制命令（含目标位置）。
     * @param[in] dt 时间步长 (s)。
     * @return 控制器输出。
     *
     * @note 积分采用条件 anti-windup：仅当积分值未超限且总输出未饱和时才累积，防止积分过冲。
     */
    ControllerOutput update(const VehicleState& vehicle, const ControllerCommand& command, double dt);

private:
    PidAxis pid_x_;                /**< X 轴 PID */
    PidAxis pid_y_;                /**< Y 轴 PID */
    PidAxis pid_z_;                /**< Z 轴 PID */
    PidAxis pid_yaw_;              /**< 偏航轴 PID */

    double max_xy_speed_ = 1.0;            /**< XY 速度输出限幅 (m/s) */
    double max_z_up_speed_ = 0.6;          /**< 最大上升速度 (m/s) */
    double max_z_down_speed_ = 0.6;        /**< 最大下降速度 (m/s)，正数 */
    double max_yaw_rate_ = 0.5;            /**< 偏航角速率输出限幅 (rad/s) */
    double converge_radius_ = 0.4;         /**< 收敛判定半径 (m) */

    /**
     * @brief 角度归一化到 [-π, π)。
     * @param[in] angle 输入角度 (rad)。
     * @return 归一化后角度。
     */
    static double normalizeAngle(double angle);
};
