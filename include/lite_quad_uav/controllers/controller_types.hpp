/**
 * @file controller_types.hpp
 * @brief 控制器库公共数据结构。
 */

#pragma once

#include <string>

/* ControllerOutput: 控制器内部输出结构 ----------------------------------------*/

/**
 * @brief 控制器统一输出结构：速度候选量 + 状态标志 + 误差指标。
 */
struct ControllerOutput {
    bool valid = false;                 /**< 输出是否有效 */

    double vx = 0.0;                   /**< X 轴速度候选量 (m/s) */
    double vy = 0.0;                   /**< Y 轴速度候选量 (m/s) */
    double vz = 0.0;                   /**< Z 轴速度候选量 (m/s) */
    double yaw_rate = 0.0;             /**< 偏航角速率候选量 (rad/s) */

    double position_error_norm = 0.0;  /**< 位置误差范数 (m) */
    double target_distance = 0.0;      /**< 目标距离 (m) */
    double rl_weight = 0.0;            /**< RL 混合权重 (alpha/beta) */

    bool saturated = false;            /**< 输出是否被限幅 */
    bool converged = false;            /**< 位置是否已收敛到目标 */
    bool fallback_active = false;      /**< RL 不可用时是否回退到纯 PID */

    std::string controller_name;       /**< 控制器标识名 */
    std::string message;               /**< 补充说明信息 */
};
