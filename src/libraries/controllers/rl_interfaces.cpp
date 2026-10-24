/**
 * @file rl_interfaces.cpp
 * @brief RL 缓存接口实现：追踪策略和降落策略的输出缓存与超时管理。
 *
 * @note RLTrackingInterface 与 RLLandingInterface 实现完全相同，
 *       后续可考虑模板化或合并为单一类以消除代码重复。
 */

#include <lite_quad_uav/controllers/rl_interfaces.hpp>

/* RLTrackingInterface ------------------------------------------------------*/

/**
 * @brief 构造函数，初始化默认超时和接收标志。
 */
RLTrackingInterface::RLTrackingInterface() {
    timeout_ = 0.15;
    received_ = false;
}

/**
 * @brief 设置输出超时阈值。
 * @param[in] timeout 超时时间 (s)。
 */
void RLTrackingInterface::setTimeout(double timeout) {
    timeout_ = timeout;
}

/**
 * @brief 缓存最新 RL 追踪输出并刷新时间戳。
 * @param[in] msg RL 速度命令。
 */
void RLTrackingInterface::update(const lite_quad_uav::VelocityCommand& msg) {
    latest_ = msg;
    last_time_ = ros::Time::now();
    received_ = true;
}

/**
 * @brief 检查 RL 追踪输出是否有效（已接收 + valid 标志 + 未超时）。
 * @param[in] now 当前时间。
 * @return true=有效，false=无效或超时。
 */
bool RLTrackingInterface::isValid(const ros::Time& now) const {
    if (!received_) return false;
    if (!latest_.valid) return false;
    return (now - last_time_).toSec() <= timeout_;
}

/**
 * @brief 获取最新缓存的 RL 追踪速度命令。
 * @return 最近一次接收到的速度命令。
 */
lite_quad_uav::VelocityCommand RLTrackingInterface::latest() const {
    return latest_;
}

/**
 * @brief 清除缓存，重置为未接收状态。
 */
void RLTrackingInterface::reset() {
    received_ = false;
    latest_ = lite_quad_uav::VelocityCommand();
}

/* RLLandingInterface -------------------------------------------------------*/

/**
 * @brief 构造函数，初始化默认超时和接收标志。
 */
RLLandingInterface::RLLandingInterface() {
    timeout_ = 0.15;
    received_ = false;
}

/**
 * @brief 设置输出超时阈值。
 * @param[in] timeout 超时时间 (s)。
 */
void RLLandingInterface::setTimeout(double timeout) {
    timeout_ = timeout;
}

/**
 * @brief 缓存最新 RL 降落输出并刷新时间戳。
 * @param[in] msg RL 速度命令。
 */
void RLLandingInterface::update(const lite_quad_uav::VelocityCommand& msg) {
    latest_ = msg;
    last_time_ = ros::Time::now();
    received_ = true;
}

/**
 * @brief 检查 RL 降落输出是否有效（已接收 + valid 标志 + 未超时）。
 * @param[in] now 当前时间。
 * @return true=有效，false=无效或超时。
 */
bool RLLandingInterface::isValid(const ros::Time& now) const {
    if (!received_) return false;
    if (!latest_.valid) return false;
    return (now - last_time_).toSec() <= timeout_;
}

/**
 * @brief 获取最新缓存的 RL 降落速度命令。
 * @return 最近一次接收到的速度命令。
 */
lite_quad_uav::VelocityCommand RLLandingInterface::latest() const {
    return latest_;
}

/**
 * @brief 清除缓存，重置为未接收状态。
 */
void RLLandingInterface::reset() {
    received_ = false;
    latest_ = lite_quad_uav::VelocityCommand();
}
