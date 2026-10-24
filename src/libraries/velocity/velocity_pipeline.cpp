/**
 * @file velocity_pipeline.cpp
 * @brief VelocityPipeline 实现：Watchdog / 安全保持 / 速度限幅 / 安全过滤。
 *
 * 处理链顺序为 applyWatchdog → applySafetyFilter → applyLimits，
 * 任何环节异常均降级为 SAFE_HOLD 零速度输出。
 */

#include <lite_quad_uav/velocity/velocity_pipeline.hpp>
#include <cmath>

/* VelocityPipeline 实现 -----------------------------------------------------*/

/**
 * @brief 构造函数，初始化默认参数和管道状态。
 */
VelocityPipeline::VelocityPipeline() {
    params_.publish_rate = 50.0;
    params_.command_timeout = 0.5;
    params_.max_xy_speed = 1.0;
    params_.max_z_up_speed = 0.6;
    params_.max_z_down_speed = 0.6;
    params_.max_yaw_rate = 1.0;

    pipeline_status_.state = "SAFE_HOLD";
    pipeline_status_.watchdog_active = true;
    pipeline_status_.command_source = "";
    pipeline_status_.message = "No command received";
}

/**
 * @brief 更新管道参数。
 * @param[in] params 管道参数。
 */
void VelocityPipeline::setParams(const PipelineParams& params) {
    params_ = params;
}

/**
 * @brief 接收新的候选速度命令，刷新看门狗计时。
 * @param[in] cmd 来自控制器层的候选速度。
 */
void VelocityPipeline::updateCommand(const lite_quad_uav::VelocityCommand& cmd) {
    candidate_ = cmd;
    last_command_time_ = ros::Time::now();
    command_received_ = true;
}

/**
 * @brief 更新安全监督状态，供安全过滤环节使用。
 * @param[in] status 安全监督状态。
 */
void VelocityPipeline::updateSafetyStatus(const lite_quad_uav::SafetyStatus& status) {
    safety_status_ = status;
}

/**
 * @brief 数值限幅。
 * @param[in] val 待限幅值。
 * @param[in] lo 下限。
 * @param[in] hi 上限。
 * @return 限幅后值。
 */
double VelocityPipeline::clamp(double val, double lo, double hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief 看门狗检查：未收到命令或命令超时则降级为 SAFE_HOLD。
 * @param[in] now 当前时间。
 * @return 有效命令或 SAFE_HOLD 零速度。
 */
lite_quad_uav::VelocityCommand VelocityPipeline::applyWatchdog(const ros::Time& now) {
    if (!command_received_) {
        pipeline_status_.watchdog_active = true;
        pipeline_status_.state = "SAFE_HOLD";
        return applySafeHold();
    }

    double age = (now - last_command_time_).toSec();
    pipeline_status_.last_command_age_sec = age;

    if (age > params_.command_timeout) {
        pipeline_status_.watchdog_active = true;
        pipeline_status_.state = "SAFE_HOLD";
        pipeline_status_.message = "Command timeout";
        return applySafeHold();
    }

    pipeline_status_.watchdog_active = false;
    lite_quad_uav::VelocityCommand cmd = candidate_;
    if (!cmd.valid) {
        return applySafeHold();
    }
    return cmd;
}

/**
 * @brief 生成零速度安全保持命令，source="safe_hold"。
 * @return valid=true 的零速度命令。
 */
lite_quad_uav::VelocityCommand VelocityPipeline::applySafeHold() {
    lite_quad_uav::VelocityCommand cmd;
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.vz = 0.0;
    cmd.yaw_rate = 0.0;
    cmd.valid = true;
    cmd.source = "safe_hold";
    return cmd;
}

/**
 * @brief 对速度命令各轴做硬限幅。
 * @param[in] cmd 待限幅命令。
 * @return 限幅后命令。
 */
lite_quad_uav::VelocityCommand VelocityPipeline::applyLimits(
    const lite_quad_uav::VelocityCommand& cmd) {
    lite_quad_uav::VelocityCommand out = cmd;

    double speed_xy = std::sqrt(out.vx * out.vx + out.vy * out.vy);
    if (speed_xy > params_.max_xy_speed && speed_xy > 0.0) {
        double scale = params_.max_xy_speed / speed_xy;
        out.vx *= scale;
        out.vy *= scale;
    }

    if (out.vz > params_.max_z_up_speed) {
        out.vz = params_.max_z_up_speed;
    } else if (out.vz < -params_.max_z_down_speed) {
        out.vz = -params_.max_z_down_speed;
    }

    out.yaw_rate = clamp(out.yaw_rate, -params_.max_yaw_rate, params_.max_yaw_rate);
    return out;
}

/**
 * @brief 安全过滤：EMERGENCY 或 ERROR 级别时降级为 SAFE_HOLD。
 * @param[in] cmd 待过滤命令。
 * @return 原命令或 SAFE_HOLD 零速度。
 */
lite_quad_uav::VelocityCommand VelocityPipeline::applySafetyFilter(
    const lite_quad_uav::VelocityCommand& cmd) {
    if (safety_status_.level == "EMERGENCY" || safety_status_.level == "ERROR") {
        pipeline_status_.state = "SAFETY_FILTER";
        pipeline_status_.message = "Safety filter: " + safety_status_.level;
        return applySafeHold();
    }
    return cmd;
}

/**
 * @brief 执行完整处理链（Watchdog → SafetyFilter → Limits），返回最终速度。
 * @param[in] now 当前时间，用于超时判断。
 * @return 经管道处理后的速度命令。
 */
lite_quad_uav::VelocityCommand VelocityPipeline::process(const ros::Time& now) {
    // Step 1: Watchdog check
    lite_quad_uav::VelocityCommand cmd = applyWatchdog(now);

    // Step 2: Safety filter
    cmd = applySafetyFilter(cmd);

    // Step 3: Velocity limits
    cmd = applyLimits(cmd);

    // Update status
    if (!pipeline_status_.watchdog_active && pipeline_status_.state != "SAFETY_FILTER") {
        pipeline_status_.state = "NORMAL";
        pipeline_status_.command_source = cmd.source;
        pipeline_status_.message = "";
    }

    return cmd;
}

/**
 * @brief 获取管道内部状态。
 * @return 管道状态。
 */
lite_quad_uav::PipelineStatus VelocityPipeline::getStatus() const {
    return pipeline_status_;
}
