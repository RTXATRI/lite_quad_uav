/**
 * @file actions.cpp
 * @brief Action 动作实现：Takeoff / Waypoint / Hold / TrackTarget / Landing。
 *
 * 每种 Action 实现状态机模式，onEnter 初始化 → onUpdate 每周期计算 → onExit 清理 / onCancel 取消。
 */

#include <lite_quad_uav/tasks/actions.hpp>

#include <cmath>

/* Helper functions / 辅助函数 ------------------------------------------------*/

/**
 * @brief 构造基础 ControlRequest，填充 header/source/mode/valid。
 * @param[in] now 时间戳。
 * @param[in] source_task 任务来源标识。
 * @param[in] source_action 动作名称。
 * @param[in] mode 控制模式。
 * @param[in] valid 请求是否有效。
 * @return 填充后的 ControlRequest。
 */
static lite_quad_uav::ControlRequest make_base_request(const ros::Time& now, const std::string& source_task, const std::string& source_action, const std::string& mode, bool valid) {
    lite_quad_uav::ControlRequest request;
    request.header.stamp = now;
    request.source_task = source_task;
    request.source_action = source_action;
    request.mode = mode;
    request.reference_velocity_ff.x = 0.0;
    request.reference_velocity_ff.y = 0.0;
    request.reference_velocity_ff.z = 0.0;
    request.reference_yaw = 0.0;
    request.valid = valid;
    return request;
}

/**
 * @brief 计算飞行器与航点的三维欧氏距离。
 * @param[in] vehicle_state 飞行器状态。
 * @param[in] waypoint 目标航点。
 * @return 距离 (m)。
 */
static double distance_3d(const VehicleState& vehicle_state, const Waypoint& waypoint) {
    double dx = vehicle_state.x - waypoint.x;
    double dy = vehicle_state.y - waypoint.y;
    double dz = vehicle_state.z - waypoint.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/* TakeoffAction -------------------------------------------------------------*/

/**
 * @brief 构造起飞动作。
 * @param[in] config 起飞参数配置。
 */
TakeoffAction::TakeoffAction(const TakeoffActionConfig& config) : config_(config) {
}

/**
 * @brief 起飞动作进入，初始化内部状态。
 * @param[in] now 当前时间。
 */
void TakeoffAction::onEnter(const ros::Time& /*now*/) {
    status_ = ActionStatus::RUNNING;
    target_locked_ = false;
    stable_started_ = false;
}

/**
 * @brief 起飞逻辑：锁定起飞位置 → 等待高度到达 → 稳定计时后 SUCCESS。
 * @param[in] input 动作输入。
 * @return 动作输出（控制请求 + 动作状态）。
 */
ActionOutput TakeoffAction::onUpdate(const ActionInput& input) {
    ActionOutput output;
    output.status = status_;

    // 非运行状态返回无效请求
    if (status_ != ActionStatus::RUNNING) {
        output.control_request = makeRequest(input.now, false);
        return output;
    }

    // 等待有效飞行器状态
    if (!input.vehicle_state.valid) {
        ROS_WARN_THROTTLE(2.0, "[TakeoffAction] Waiting for valid vehicle state");
        output.status = ActionStatus::RUNNING;
        output.control_request = makeRequest(input.now, false);
        return output;
    }

    // 锁定起飞 XY 位置
    if (!target_locked_) {
        takeoff_x_ = input.vehicle_state.x;
        takeoff_y_ = input.vehicle_state.y;
        target_locked_ = true;
        ROS_INFO("[TakeoffAction] Target locked, x = %.2f, y = %.2f, z = %.2f", takeoff_x_, takeoff_y_, config_.takeoff_height);
    }

    // 高度到达判定 + 稳态保持计时
    double height_error = std::fabs(input.vehicle_state.z - config_.takeoff_height);
    if (height_error < config_.height_accept_error) {
        if (!stable_started_) {
            stable_started_ = true;
            stable_start_time_ = input.now;
        }
        if ((input.now - stable_start_time_).toSec() >= config_.stable_time) {
            status_ = ActionStatus::SUCCESS;
        }
    } else {
        stable_started_ = false;
    }

    output.status = status_;
    output.control_request = makeRequest(input.now, true);
    return output;
}

/**
 * @brief 起飞动作退出。
 * @param[in] now 当前时间。
 */
void TakeoffAction::onExit(const ros::Time& /*now*/) {
}

/**
 * @brief 起飞动作取消。
 * @param[in] now 当前时间。
 */
void TakeoffAction::onCancel(const ros::Time& /*now*/) {
    status_ = ActionStatus::CANCELED;
}

/**
 * @brief 构造起飞 ControlRequest：锁定 XY + 目标高度。
 * @param[in] now 时间戳。
 * @param[in] valid 请求是否有效。
 * @return 填充后的 ControlRequest。
 */
lite_quad_uav::ControlRequest TakeoffAction::makeRequest(const ros::Time& now, bool valid) const {
    lite_quad_uav::ControlRequest request = make_base_request(now, config_.source_task, "takeoff", valid ? "REFERENCE_TRACKING" : "IDLE", valid);
    request.reference_position.x = takeoff_x_;
    request.reference_position.y = takeoff_y_;
    request.reference_position.z = config_.takeoff_height;
    request.max_xy_speed = config_.max_xy_speed;
    request.max_z_up_speed = config_.max_z_up_speed;
    request.max_z_down_speed = config_.max_z_down_speed;
    request.max_yaw_rate = config_.max_yaw_rate;
    return request;
}

/* WaypointAction ------------------------------------------------------------*/

/**
 * @brief 构造巡航动作。
 * @param[in] config 巡航参数配置。
 */
WaypointAction::WaypointAction(const WaypointActionConfig& config) : config_(config) {
}

/**
 * @brief 巡航动作进入，初始化内部状态。
 * @param[in] now 当前时间。
 */
void WaypointAction::onEnter(const ros::Time& /*now*/) {
    status_ = ActionStatus::RUNNING;
    current_index_ = 0;
    hold_started_ = false;
}

/**
 * @brief 巡航逻辑：依次到达航点，到达后停留 hold_time，loop=true 时循环。
 * @param[in] input 动作输入。
 * @return 动作输出（控制请求 + 动作状态）。
 */
ActionOutput WaypointAction::onUpdate(const ActionInput& input) {
    ActionOutput output;
    output.status = status_;

    // 非运行状态返回无效请求
    if (status_ != ActionStatus::RUNNING) {
        output.control_request = makeRequest(input.now, false);
        return output;
    }

    // 航点列表为空则失败
    if (config_.waypoints.empty()) {
        ROS_ERROR_THROTTLE(2.0, "[WaypointAction] No waypoints configured");
        status_ = ActionStatus::FAILED;
        output.status = status_;
        output.control_request = makeRequest(input.now, false);
        return output;
    }

    // 等待有效飞行器状态
    if (!input.vehicle_state.valid) {
        ROS_WARN_THROTTLE(2.0, "[WaypointAction] Waiting for valid vehicle state");
        output.status = ActionStatus::RUNNING;
        output.control_request = makeRequest(input.now, false);
        return output;
    }

    // 航点到达判定
    const Waypoint& waypoint = config_.waypoints[current_index_];
    double distance = distance_3d(input.vehicle_state, waypoint);
    if (distance < config_.accept_radius) {
        if (!hold_started_) {
            hold_started_ = true;
            hold_start_time_ = input.now;
        }

        if ((input.now - hold_start_time_).toSec() >= config_.hold_time) {
            if (current_index_ + 1 < config_.waypoints.size()) {
                current_index_++;
                hold_started_ = false;
                ROS_INFO("[WaypointAction] Moving to waypoint %zu", current_index_);
            } else if (config_.loop) {
                current_index_ = 0;
                hold_started_ = false;
                ROS_INFO("[WaypointAction] Looping to waypoint 0");
            } else {
                status_ = ActionStatus::SUCCESS;
            }
        }
    } else {
        hold_started_ = false;
    }

    output.status = status_;
    output.control_request = makeRequest(input.now, true);
    return output;
}

/**
 * @brief 巡航动作退出。
 * @param[in] now 当前时间。
 */
void WaypointAction::onExit(const ros::Time& /*now*/) {
}

/**
 * @brief 巡航动作取消。
 * @param[in] now 当前时间。
 */
void WaypointAction::onCancel(const ros::Time& /*now*/) {
    status_ = ActionStatus::CANCELED;
}

/**
 * @brief 构造巡航 ControlRequest：当前航点位置。
 * @param[in] now 时间戳。
 * @param[in] valid 请求是否有效。
 * @return 填充后的 ControlRequest。
 */
lite_quad_uav::ControlRequest WaypointAction::makeRequest(const ros::Time& now, bool valid) const {
    lite_quad_uav::ControlRequest request = make_base_request(now, config_.source_task, "waypoint", valid ? "REFERENCE_TRACKING" : "IDLE", valid);
    if (!config_.waypoints.empty() && current_index_ < config_.waypoints.size()) {
        request.reference_position.x = config_.waypoints[current_index_].x;
        request.reference_position.y = config_.waypoints[current_index_].y;
        request.reference_position.z = config_.waypoints[current_index_].z;
    }
    request.max_xy_speed = config_.max_xy_speed;
    request.max_z_up_speed = config_.max_z_up_speed;
    request.max_z_down_speed = config_.max_z_down_speed;
    request.max_yaw_rate = config_.max_yaw_rate;
    return request;
}

/* HoldAction ----------------------------------------------------------------*/

/**
 * @brief 构造悬停动作。
 * @param[in] config 悬停参数配置。
 */
HoldAction::HoldAction(const HoldActionConfig& config) : config_(config) {
}

/**
 * @brief 悬停动作进入，初始化内部状态。
 * @param[in] now 当前时间。
 */
void HoldAction::onEnter(const ros::Time& /*now*/) {
    status_ = ActionStatus::RUNNING;
    hold_point_locked_ = false;
}

/**
 * @brief 悬停逻辑：锁定当前位置后持续输出 HOLD 请求。
 * @param[in] input 动作输入。
 * @return 动作输出（控制请求 + 动作状态）。
 */
ActionOutput HoldAction::onUpdate(const ActionInput& input) {
    ActionOutput output;
    output.status = status_;

    // 非运行状态返回无效请求
    if (status_ != ActionStatus::RUNNING) {
        output.control_request = makeRequest(input.now, false);
        return output;
    }

    // 等待有效飞行器状态
    if (!input.vehicle_state.valid) {
        ROS_WARN_THROTTLE(2.0, "[HoldAction] Waiting for valid vehicle state");
        output.status = ActionStatus::RUNNING;
        output.control_request = makeRequest(input.now, false);
        return output;
    }

    // 锁定悬停位置
    if (!hold_point_locked_) {
        hold_x_ = input.vehicle_state.x;
        hold_y_ = input.vehicle_state.y;
        hold_z_ = input.vehicle_state.z;
        hold_point_locked_ = true;
        ROS_INFO("[HoldAction] Hold point locked, x = %.2f, y = %.2f, z = %.2f", hold_x_, hold_y_, hold_z_);
    }

    output.status = ActionStatus::RUNNING;
    output.control_request = makeRequest(input.now, true);
    return output;
}

/**
 * @brief 悬停动作退出。
 * @param[in] now 当前时间。
 */
void HoldAction::onExit(const ros::Time& /*now*/) {
}

/**
 * @brief 悬停动作取消。
 * @param[in] now 当前时间。
 */
void HoldAction::onCancel(const ros::Time& /*now*/) {
    status_ = ActionStatus::CANCELED;
}

/**
 * @brief 构造悬停 ControlRequest：锁定位置。
 * @param[in] now 时间戳。
 * @param[in] valid 请求是否有效。
 * @return 填充后的 ControlRequest。
 */
lite_quad_uav::ControlRequest HoldAction::makeRequest(const ros::Time& now, bool valid) const {
    lite_quad_uav::ControlRequest request = make_base_request(now, config_.source_task, "hold", valid ? "HOLD" : "IDLE", valid);
    request.reference_position.x = hold_x_;
    request.reference_position.y = hold_y_;
    request.reference_position.z = hold_z_;
    request.max_xy_speed = config_.max_xy_speed;
    request.max_z_up_speed = config_.max_z_up_speed;
    request.max_z_down_speed = config_.max_z_down_speed;
    request.max_yaw_rate = config_.max_yaw_rate;
    return request;
}

/* Placeholder actions / 占位动作 --------------------------------------------*/

/**
 * @brief 构造目标追踪动作（占位实现）。
 * @param[in] source_task 任务来源标识。
 */
TrackTargetAction::TrackTargetAction(const std::string& source_task) : source_task_(source_task) {
}

/**
 * @brief 目标追踪动作进入（占位）。
 */
void TrackTargetAction::onEnter(const ros::Time& /*now*/) {
}

/**
 * @brief 目标追踪动作更新（占位，当前始终返回 FAILED）。
 * @param[in] input 动作输入。
 * @return 动作输出（FAILED 状态）。
 *
 * @note 当前为占位实现，等目标状态接口明确后再实现。
 */
ActionOutput TrackTargetAction::onUpdate(const ActionInput& input) {
    // TODO: 等目标状态接口明确后再实现目标跟踪。
    ActionOutput output;
    output.status = ActionStatus::FAILED;
    output.control_request = make_base_request(input.now, source_task_, "track_target", "TARGET_TRACKING", false);
    return output;
}

void TrackTargetAction::onExit(const ros::Time& /*now*/) {
}

void TrackTargetAction::onCancel(const ros::Time& /*now*/) {
}

/**
 * @brief 构造降落动作（占位实现）。
 * @param[in] source_task 任务来源标识。
 */
LandingAction::LandingAction(const std::string& source_task) : source_task_(source_task) {
}

/**
 * @brief 降落动作进入（占位）。
 */
void LandingAction::onEnter(const ros::Time& /*now*/) {
}

/**
 * @brief 降落动作更新（占位，当前始终返回 FAILED）。
 * @param[in] input 动作输入。
 * @return 动作输出（FAILED 状态）。
 *
 * @note 当前为占位实现，等降落策略明确后再实现。
 */
ActionOutput LandingAction::onUpdate(const ActionInput& input) {
    // TODO: 等降落策略明确后再实现降落动作。
    ActionOutput output;
    output.status = ActionStatus::FAILED;
    output.control_request = make_base_request(input.now, source_task_, "landing", "LANDING", false);
    return output;
}

void LandingAction::onExit(const ros::Time& /*now*/) {
}

void LandingAction::onCancel(const ros::Time& /*now*/) {
}
