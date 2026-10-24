/**
 * @file velocity_pipeline.hpp
 * @brief 速度管道：Watchdog + 安全过滤 + 速度限幅处理链。
 *
 * 管道输出最终由 velocity_manager_node 发布到 MAVROS，确保任何输入异常都降级为零速度安全保持。
 */

#pragma once

#include <ros/ros.h>
#include <lite_quad_uav/VelocityCommand.h>
#include <lite_quad_uav/SafetyStatus.h>
#include <lite_quad_uav/PipelineStatus.h>

/**
 * @brief 速度管道参数配置。
 */
struct PipelineParams {
    double publish_rate;       /**< 发布频率 (Hz) */
    double command_timeout;    /**< 命令超时阈值 (s) */
    double max_xy_speed;       /**< XY 速度模长限幅 (m/s) */
    double max_z_up_speed;     /**< 最大上升速度 (m/s) */
    double max_z_down_speed;   /**< 最大下降速度 (m/s)，正数 */
    double max_yaw_rate;       /**< 偏航最大角速率 (rad/s) */
};

/**
 * @brief 速度候选量处理管道：超时看门狗 → 安全过滤 → 速度限幅。
 *
 */
class VelocityPipeline {
public:
    /**
     * @brief 构造函数，初始化默认参数和管道状态。
     */
    VelocityPipeline();

    /**
     * @brief 更新管道参数。
     * @param[in] params 管道参数。
     */
    void setParams(const PipelineParams& params);

    /**
     * @brief 接收新的候选速度命令，刷新看门狗计时。
     * @param[in] cmd 来自控制器层的候选速度。
     */
    void updateCommand(const lite_quad_uav::VelocityCommand& cmd);

    /**
     * @brief 更新安全监督状态，供安全过滤环节使用。
     * @param[in] status 安全监督状态。
     */
    void updateSafetyStatus(const lite_quad_uav::SafetyStatus& status);

    /**
     * @brief 执行完整处理链（Watchdog → SafetyFilter → Limits），返回最终速度。
     * @param[in] now 当前时间，用于超时判断。
     * @return 经管道处理后的速度命令。
     */
    lite_quad_uav::VelocityCommand process(const ros::Time& now);

    /**
     * @brief 获取管道内部状态。
     * @return 管道状态。
     */
    lite_quad_uav::PipelineStatus getStatus() const;

private:
    PipelineParams params_;                              /**< 管道参数 */
    lite_quad_uav::VelocityCommand candidate_;           /**< 最新候选速度命令 */
    lite_quad_uav::SafetyStatus safety_status_;          /**< 最新安全监督状态 */
    lite_quad_uav::PipelineStatus pipeline_status_;      /**< 管道内部状态 */
    ros::Time last_command_time_;                         /**< 上次收到命令的时间 */
    bool command_received_ = false;                      /**< 是否已收到至少一次命令 */

    /**
     * @brief 看门狗检查：未收到命令或命令超时则降级为 SAFE_HOLD。
     * @param[in] now 当前时间。
     * @return 有效命令或 SAFE_HOLD 零速度。
     */
    lite_quad_uav::VelocityCommand applyWatchdog(const ros::Time& now);

    /**
     * @brief 生成零速度安全保持命令，source="safe_hold"。
     * @return valid=true 的零速度命令。
     */
    lite_quad_uav::VelocityCommand applySafeHold();

    /**
     * @brief 对速度命令各轴做硬限幅。
     * @param[in] cmd 待限幅命令。
     * @return 限幅后命令。
     */
    lite_quad_uav::VelocityCommand applyLimits(const lite_quad_uav::VelocityCommand& cmd);

    /**
     * @brief 安全过滤：EMERGENCY 或 ERROR 级别时降级为 SAFE_HOLD。
     * @param[in] cmd 待过滤命令。
     * @return 原命令或 SAFE_HOLD 零速度。
     */
    lite_quad_uav::VelocityCommand applySafetyFilter(const lite_quad_uav::VelocityCommand& cmd);

    /**
     * @brief 数值限幅。
     * @param[in] val 待限幅值。
     * @param[in] lo 下限。
     * @param[in] hi 上限。
     * @return 限幅后值。
     */
    double clamp(double val, double lo, double hi);
};
