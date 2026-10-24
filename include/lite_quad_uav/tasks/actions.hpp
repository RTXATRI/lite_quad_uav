/**
 * @file actions.hpp
 * @brief Action 动作基类及五种具体动作定义：Takeoff / Waypoint / Hold / TrackTarget / Landing。
 *
 * 动作实现状态机模式 (onEnter/onUpdate/onExit/onCancel)，由 Task 层调度。
 * onUpdate 每周期返回 ActionOutput（含 ControlRequest），驱动 Controller 层执行。
 */

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <lite_quad_uav/ControlRequest.h>

/**
 * @brief 动作执行状态枚举。
 */
enum class ActionStatus {
    IDLE = 0,       /**< 空闲，尚未启动 */
    RUNNING,        /**< 运行中 */
    SUCCESS,        /**< 已成功完成 */
    FAILED,         /**< 执行失败 */
    CANCELED,        /**< 被外部取消 */
    TIMEOUT         /**< 执行超时 */
};

/**
 * @brief 飞行器当前状态：位置 + 速度。
 */
struct VehicleState {
    bool valid = false;         /**< 数据是否有效 */
    double x = 0.0;            /**< X 坐标 (m)，ENU 坐标系 */
    double y = 0.0;            /**< Y 坐标 (m)，ENU 坐标系 */
    double z = 0.0;            /**< Z 坐标 (m)，ENU 坐标系（向上为正） */
    double yaw = 0.0;          /**< 偏航角 (rad) */
    double vx = 0.0;           /**< X 轴速度 (m/s) */
    double vy = 0.0;           /**< Y 轴速度 (m/s) */
    double vz = 0.0;           /**< Z 轴速度 (m/s) */
    double yaw_rate = 0.0;     /**< 偏航角速率 (rad/s) */
};

/**
 * @brief 控制器反馈数据：收敛状态和位置误差。
 */
struct ControllerFeedbackData {
    bool valid = false;                    /**< 反馈数据是否有效 */
    bool converged = false;               /**< 位置是否已收敛到目标 */
    double position_error_norm = 0.0;     /**< 位置误差范数 (m) */
};

/**
 * @brief 动作输入：当前时间 + 飞行器状态 + 控制器反馈。
 */
struct ActionInput {
    ros::Time now;                            /**< 当前时间 */
    VehicleState vehicle_state;               /**< 飞行器状态 */
    ControllerFeedbackData controller_feedback; /**< 控制器反馈 */
};

/**
 * @brief 动作输出：动作状态 + 控制请求。
 */
struct ActionOutput {
    ActionStatus status = ActionStatus::IDLE;       /**< 当前动作状态 */
    lite_quad_uav::ControlRequest control_request;  /**< 输出给 Controller 层的控制请求 */
};

/**
 * @brief 三维航点。
 */
struct Waypoint {
    Waypoint() {}
    Waypoint(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

    double x = 0.0;     /**< X 坐标 (m) */
    double y = 0.0;     /**< Y 坐标 (m) */
    double z = 0.0;     /**< Z 坐标 (m) */
};

/**
 * @brief 起飞动作参数。
 */
struct TakeoffActionConfig {
    std::string source_task = "square_patrol";   /**< 任务来源标识 */
    double takeoff_height = 3.0;                 /**< 目标起飞高度 (m) */
    double height_accept_error = 0.4;            /**< 高度到达判定容差 (m) */
    double stable_time = 1.0;                    /**< 高度到达后稳态保持时间 (s) */
    double max_xy_speed = 1.0;                   /**< 水平最大速度 (m/s) */
    double max_z_up_speed = 0.6;                 /**< 最大上升速度 (m/s) */
    double max_z_down_speed = 0.6;               /**< 最大下降速度 (m/s)，正数 */
    double max_yaw_rate = 0.0;                   /**< 偏航最大角速率 (rad/s)，0=不控制偏航 */
};

/**
 * @brief 巡航动作参数。
 */
struct WaypointActionConfig {
    std::string source_task = "square_patrol";   /**< 任务来源标识 */
    std::vector<Waypoint> waypoints;             /**< 巡航航点序列 */
    double accept_radius = 0.4;                  /**< 航点到达判定半径 (m) */
    double hold_time = 0.0;                      /**< 到达航点后停留时间 (s) */
    bool loop = true;                            /**< true=循环航点，false=到达末尾后 SUCCESS */
    double max_xy_speed = 1.0;                   /**< 水平最大速度 (m/s) */
    double max_z_up_speed = 0.6;                 /**< 最大上升速度 (m/s) */
    double max_z_down_speed = 0.6;               /**< 最大下降速度 (m/s)，正数 */
    double max_yaw_rate = 0.0;                   /**< 偏航最大角速率 (rad/s) */
};

/**
 * @brief 悬停动作参数。
 */
struct HoldActionConfig {
    std::string source_task = "square_patrol";   /**< 任务来源标识 */
    double max_xy_speed = 1.0;                   /**< 水平最大速度 (m/s) */
    double max_z_up_speed = 0.6;                 /**< 最大上升速度 (m/s) */
    double max_z_down_speed = 0.6;               /**< 最大下降速度 (m/s)，正数 */
    double max_yaw_rate = 0.0;                   /**< 偏航最大角速率 (rad/s) */
};

/**
 * @brief 动作基类接口，定义 onEnter/onUpdate/onExit/onCancel 生命周期。
 *
 * Task 层通过此接口调度动作：onEnter 初始化 → onUpdate 每周期执行 → onExit 正常退出 / onCancel 取消。
 */
class ActionBase {
public:
    virtual ~ActionBase() {}

    /**
     * @brief 动作进入，初始化内部状态。
     * @param[in] now 当前时间。
     */
    virtual void onEnter(const ros::Time& now) = 0;

    /**
     * @brief 动作每周期执行，返回控制请求和动作状态。
     * @param[in] input 动作输入（时间 + 飞行器状态 + 控制器反馈）。
     * @return 动作输出（状态 + 控制请求）。
     */
    virtual ActionOutput onUpdate(const ActionInput& input) = 0;

    /**
     * @brief 动作正常退出，清理资源。
     * @param[in] now 当前时间。
     */
    virtual void onExit(const ros::Time& now) = 0;

    /**
     * @brief 动作被外部取消，立即清理。
     * @param[in] now 当前时间。
     */
    virtual void onCancel(const ros::Time& now) = 0;
};

/**
 * @brief 起飞动作：锁定起飞位置 → 等待高度到达 → 稳定计时后 SUCCESS。
 */
class TakeoffAction : public ActionBase {
public:
    /**
     * @brief 构造起飞动作。
     * @param[in] config 起飞参数配置。
     */
    explicit TakeoffAction(const TakeoffActionConfig& config);

    void onEnter(const ros::Time& now) override;
    ActionOutput onUpdate(const ActionInput& input) override;
    void onExit(const ros::Time& now) override;
    void onCancel(const ros::Time& now) override;

private:
    TakeoffActionConfig config_;         /**< 起飞参数 */
    ActionStatus status_ = ActionStatus::IDLE;   /**< 当前动作状态 */
    bool target_locked_ = false;         /**< 起飞位置是否已锁定 */
    bool stable_started_ = false;        /**< 稳态计时是否已开始 */
    double takeoff_x_ = 0.0;            /**< 锁定的起飞 X 坐标 (m) */
    double takeoff_y_ = 0.0;            /**< 锁定的起飞 Y 坐标 (m) */
    ros::Time stable_start_time_;        /**< 稳态计时起始时间 */

    /**
     * @brief 构造 ControlRequest，填充 header/source/mode/valid。
     * @param[in] now 时间戳。
     * @param[in] valid 请求是否有效。
     * @return 填充后的 ControlRequest。
     */
    lite_quad_uav::ControlRequest makeRequest(const ros::Time& now, bool valid) const;
};

/**
 * @brief 巡航动作：依次到达航点，到达后停留 hold_time，loop=true 时循环。
 */
class WaypointAction : public ActionBase {
public:
    /**
     * @brief 构造巡航动作。
     * @param[in] config 巡航参数配置。
     */
    explicit WaypointAction(const WaypointActionConfig& config);

    void onEnter(const ros::Time& now) override;
    ActionOutput onUpdate(const ActionInput& input) override;
    void onExit(const ros::Time& now) override;
    void onCancel(const ros::Time& now) override;

private:
    WaypointActionConfig config_;        /**< 巡航参数 */
    ActionStatus status_ = ActionStatus::IDLE;   /**< 当前动作状态 */
    size_t current_index_ = 0;           /**< 当前目标航点索引 */
    bool hold_started_ = false;          /**< 航点停留计时是否已开始 */
    ros::Time hold_start_time_;          /**< 停留计时起始时间 */

    /**
     * @brief 构造 ControlRequest，填充 header/source/mode/valid。
     * @param[in] now 时间戳。
     * @param[in] valid 请求是否有效。
     * @return 填充后的 ControlRequest。
     */
    lite_quad_uav::ControlRequest makeRequest(const ros::Time& now, bool valid) const;
};

/**
 * @brief 悬停动作：锁定当前位置后持续输出 HOLD 请求。
 */
class HoldAction : public ActionBase {
public:
    /**
     * @brief 构造悬停动作。
     * @param[in] config 悬停参数配置。
     */
    explicit HoldAction(const HoldActionConfig& config);

    void onEnter(const ros::Time& now) override;
    ActionOutput onUpdate(const ActionInput& input) override;
    void onExit(const ros::Time& now) override;
    void onCancel(const ros::Time& now) override;

private:
    HoldActionConfig config_;            /**< 悬停参数 */
    ActionStatus status_ = ActionStatus::IDLE;   /**< 当前动作状态 */
    bool hold_point_locked_ = false;     /**< 悬停位置是否已锁定 */
    double hold_x_ = 0.0;               /**< 锁定的悬停 X 坐标 (m) */
    double hold_y_ = 0.0;               /**< 锁定的悬停 Y 坐标 (m) */
    double hold_z_ = 0.0;               /**< 锁定的悬停 Z 坐标 (m) */

    /**
     * @brief 构造 ControlRequest，填充 header/source/mode/valid。
     * @param[in] now 时间戳。
     * @param[in] valid 请求是否有效。
     * @return 填充后的 ControlRequest。
     */
    lite_quad_uav::ControlRequest makeRequest(const ros::Time& now, bool valid) const;
};

/**
 * @brief 目标追踪动作（占位实现，当前始终返回 FAILED）。
 */
class TrackTargetAction : public ActionBase {
public:
    /**
     * @brief 构造目标追踪动作。
     * @param[in] source_task 任务来源标识。
     */
    explicit TrackTargetAction(const std::string& source_task = "square_patrol");

    void onEnter(const ros::Time& now) override;
    ActionOutput onUpdate(const ActionInput& input) override;
    void onExit(const ros::Time& now) override;
    void onCancel(const ros::Time& now) override;

private:
    std::string source_task_;            /**< 任务来源标识 */
};

/**
 * @brief 降落动作（占位实现，当前始终返回 FAILED）。
 */
class LandingAction : public ActionBase {
public:
    /**
     * @brief 构造降落动作。
     * @param[in] source_task 任务来源标识。
     */
    explicit LandingAction(const std::string& source_task = "square_patrol");

    void onEnter(const ros::Time& now) override;
    ActionOutput onUpdate(const ActionInput& input) override;
    void onExit(const ros::Time& now) override;
    void onCancel(const ros::Time& now) override;

private:
    std::string source_task_;            /**< 任务来源标识 */
};
