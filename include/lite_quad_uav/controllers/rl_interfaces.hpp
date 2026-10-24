/**
 * @file rl_interfaces.hpp
 * @brief RL 输出缓存接口：追踪策略和降落策略的输出缓存与超时管理。
 */

#pragma once

#include <ros/ros.h>

#include <lite_quad_uav/VelocityCommand.h>

/* RLTrackingInterface: RL 追踪输出缓存器 ------------------------------------*/

/**
 * @brief RL 追踪策略输出缓存器：超时自动失效。
 *
 * @note RLTrackingInterface 与 RLLandingInterface 当前实现完全相同，
 *       后续版本可考虑统一为模板类以消除重复。
 */
class RLTrackingInterface {
public:
    /**
     * @brief 构造函数，初始化默认超时和接收标志。
     */
    RLTrackingInterface();

    /**
     * @brief 设置输出超时阈值。
     * @param[in] timeout 超时时间 (s)。
     */
    void setTimeout(double timeout);

    /**
     * @brief 缓存最新 RL 追踪输出并刷新时间戳。
     * @param[in] msg RL 速度命令。
     */
    void update(const lite_quad_uav::VelocityCommand& msg);

    /**
     * @brief 检查 RL 追踪输出是否有效（已接收 + valid 标志 + 未超时）。
     * @param[in] now 当前时间。
     * @return true=有效，false=无效或超时。
     */
    bool isValid(const ros::Time& now) const;

    /**
     * @brief 获取最新缓存的 RL 追踪速度命令。
     * @return 最近一次接收到的速度命令。
     */
    lite_quad_uav::VelocityCommand latest() const;

    /**
     * @brief 清除缓存，重置为未接收状态。
     */
    void reset();

private:
    lite_quad_uav::VelocityCommand latest_;   /**< 最新缓存的速度命令 */
    ros::Time last_time_;                      /**< 上次接收时间 */
    double timeout_ = 0.15;                    /**< 输出超时阈值 (s) */
    bool received_ = false;                    /**< 是否已收到至少一次 RL 输出 */
};

/* RLLandingInterface: RL 降落输出缓存器 -------------------------------------*/

/**
 * @brief RL 降落策略输出缓存器：超时自动失效。
 *
 * @note RLTrackingInterface 与 RLLandingInterface 当前实现完全相同，
 *       后续版本可考虑统一为模板类以消除重复。
 */
class RLLandingInterface {
public:
    /**
     * @brief 构造函数，初始化默认超时和接收标志。
     */
    RLLandingInterface();

    /**
     * @brief 设置输出超时阈值。
     * @param[in] timeout 超时时间 (s)。
     */
    void setTimeout(double timeout);

    /**
     * @brief 缓存最新 RL 降落输出并刷新时间戳。
     * @param[in] msg RL 速度命令。
     */
    void update(const lite_quad_uav::VelocityCommand& msg);

    /**
     * @brief 检查 RL 降落输出是否有效（已接收 + valid 标志 + 未超时）。
     * @param[in] now 当前时间。
     * @return true=有效，false=无效或超时。
     */
    bool isValid(const ros::Time& now) const;

    /**
     * @brief 获取最新缓存的 RL 降落速度命令。
     * @return 最近一次接收到的速度命令。
     */
    lite_quad_uav::VelocityCommand latest() const;

    /**
     * @brief 清除缓存，重置为未接收状态。
     */
    void reset();

private:
    lite_quad_uav::VelocityCommand latest_;   /**< 最新缓存的速度命令 */
    ros::Time last_time_;                      /**< 上次接收时间 */
    double timeout_ = 0.15;                    /**< 输出超时阈值 (s) */
    bool received_ = false;                    /**< 是否已收到至少一次 RL 输出 */
};
