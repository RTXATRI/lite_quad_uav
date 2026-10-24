/**
 * @file controller_node.cpp
 * @brief 控制器调度节点：根据 ControlRequest 生成候选速度 VelocityCommand。
 *
 * 本节点只生成候选速度，不直接发布 MAVROS setpoint。
 * 最终限幅、安全过滤和 0 速度兜底由 velocity_manager_node 处理。
 * 任务层统一发布 /uav/control/request；本节点内部转换为 ControllerCommand 复用 PID 控制库。
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <lite_quad_uav/ControlRequest.h>
#include <lite_quad_uav/ControllerFeedback.h>
#include <lite_quad_uav/ControllerLogReport.h>
#include <lite_quad_uav/TargetState.h>
#include <lite_quad_uav/VehicleStatus.h>
#include <lite_quad_uav/VelocityCommand.h>
#include <lite_quad_uav/controllers/controller_lib.hpp>

/* Private typedef / 私有类型定义 --------------------------------------------*/

/**
 * @brief PID+RL 追踪混合参数。
 */
struct TrackingMixParams {
    double rl_start_distance = 5.0;      /**< RL 开始介入的目标距离 (m) */
    double rl_full_distance = 1.0;       /**< RL 完全介入的目标距离 (m) */
    double rl_alpha_max = 0.3;           /**< RL 混合权重上限 */
    double min_target_confidence = 0.7;  /**< 目标最低置信度阈值 */
};

/**
 * @brief PID+RL 降落混合参数。
 */
struct LandingMixParams {
    double rl_alpha_max = 0.3;           /**< RL 混合权重上限 */
    double min_target_confidence = 0.8;  /**< 目标最低置信度阈值 */
};

enum class ControllerNodeState {
    INITIALIZING = 0,
    NORMAL,
    REINITIALIZING
};

struct ControllerRuntimeFlags {
    bool vehicle_status_fresh = false;
    bool vehicle_connected = false;
    bool local_position_fresh = false;
    bool reliable_vehicle_connection = false;
    bool command_fresh = false;
    bool command_valid = false;
};

/* Private variables / 私有变量 ----------------------------------------------*/

static PidPositionController pid_position_controller;   /**< PID 位置控制器实例 */
static RLTrackingInterface rl_tracking;                  /**< RL 追踪输出缓存 */
static RLLandingInterface rl_landing;                    /**< RL 降落输出缓存 */

static VehicleState vehicle_state;                       /**< 飞行器当前状态 */
static ControllerCommand latest_command;                      /**< 由 ControlRequest 转换得到的最新内部控制命令 */
static lite_quad_uav::VehicleStatus vehicle_status;      /**< connect_manager 发布的车辆状态 */
static lite_quad_uav::TargetState target_state;          /**< 最新目标状态 */
static bool command_received = false;                    /**< 是否已收到控制命令 */
static bool vehicle_status_received = false;             /**< 是否已收到车辆状态 */
static bool local_position_received = false;             /**< 是否已收到本地位姿 */
static ros::Time last_command_time;                      /**< 上次收到命令的时间 */
static ros::Time last_vehicle_status_time;               /**< 上次收到车辆状态时间 */
static ros::Time last_local_position_time;               /**< 上次收到本地位姿时间 */
static ros::Time last_pid_update_time;                   /**< 上次 PID 更新时间 */
static bool pid_context_initialized = false;             /**< PID 上下文是否已初始化 */
static std::string last_pid_source_task;                 /**< 上次 PID 的 source_task */
static std::string last_pid_source_action;               /**< 上次 PID 的 source_action */
static int last_control_type = -1;                       /**< 上次控制器类型 */
static int last_pid_mix_mode = -1;                       /**< 上次混合模式 */
static ControllerOutput last_controller_output;           /**< 最近一次控制器输出缓存 */
static bool controller_output_received = false;           /**< 是否已有控制器输出 */
static lite_quad_uav::ControllerFeedback last_feedback;   /**< 最近一次控制器反馈缓存 */
static bool feedback_received = false;                    /**< 是否已有控制器反馈 */

static double update_rate = 50.0;                        /**< 控制循环频率 (Hz) */
static double runtime_check_rate = 20.0;                 /**< 运行时健康检查频率 (Hz) */
static double command_timeout = 0.3;                     /**< 命令超时 (s) */
static double vehicle_status_timeout = 2.0;              /**< 车辆状态超时 (s) */
static double local_position_timeout = 2.0;              /**< 本地位姿超时 (s) */
static double log_report_rate = 1.0;                     /**< 日志报告发布频率 (Hz) */
static std::string mavros_ns = "/mavros";               /**< MAVROS 命名空间 */

static ControllerNodeState node_state = ControllerNodeState::INITIALIZING;
static ControllerRuntimeFlags runtime_flags;
static TrackingMixParams tracking_mix;                   /**< 追踪混合参数 */
static LandingMixParams landing_mix;                     /**< 降落混合参数 */

static ros::Subscriber control_request_sub;  /**< 控制请求订阅 */
static ros::Subscriber vehicle_status_sub;   /**< 车辆状态订阅 */
static ros::Subscriber pose_sub;             /**< 位置订阅 */
static ros::Subscriber vel_sub;              /**< 速度订阅 */
static ros::Subscriber target_state_sub;     /**< 目标状态订阅 */
static ros::Subscriber rl_tracking_sub;      /**< RL 追踪输出订阅 */
static ros::Subscriber rl_landing_sub;       /**< RL 降落输出订阅 */
static ros::Publisher candidate_vel_pub;     /**< 候选速度发布 */
static ros::Publisher feedback_pub;          /**< 控制器反馈发布 */
static ros::Publisher log_report_pub;        /**< 日志报告发布 */

/* Private function prototypes / 私有函数声明 --------------------------------*/

static const char* node_state_to_string(ControllerNodeState state);
static void transition_to(ControllerNodeState next_state);
static void update_runtime_check_flags(const ros::Time& now);
static void update_node_state(const ros::Time& now);
static void refresh_controller_context(const ros::Time& now);
static void control_request_cb(const lite_quad_uav::ControlRequest::ConstPtr& msg);
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg);
static void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
static void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
static void target_state_cb(const lite_quad_uav::TargetState::ConstPtr& msg);
static void rl_tracking_cb(const lite_quad_uav::VelocityCommand::ConstPtr& msg);
static void rl_landing_cb(const lite_quad_uav::VelocityCommand::ConstPtr& msg);
static void update_controller(const ros::Time& now);
static bool command_timeout_reached(const ros::Time& now);
static ControllerCommand control_request_to_command(const lite_quad_uav::ControlRequest& request);
static void reset_pid_state();
static void reset_pid_if_command_context_changed();
static double get_pid_dt(const ros::Time& now);
static ControllerOutput update_pid_position_with_command(const ControllerCommand& command, const ros::Time& now);
static ControllerOutput run_pid_position(const ros::Time& now);
static ControllerOutput run_rl_tracking(const ros::Time& now);
static ControllerOutput run_rl_landing(const ros::Time& now);
static ControllerOutput run_pid_rl_tracking_mix(const ros::Time& now);
static ControllerOutput run_pid_rl_landing_mix(const ros::Time& now);
static double compute_tracking_mix_weight(double distance, double confidence, bool rl_valid);
static double compute_landing_mix_weight(double confidence, bool rl_valid);
static void apply_command_velocity_limits(ControllerOutput& output);
static void publish_velocity(const ControllerOutput& output, const ros::Time& now);
static void publish_feedback(const ControllerOutput& output, const ros::Time& now);
static void publish_invalid(const std::string& reason, const ros::Time& now);
static void runtime_check_timer_cb(const ros::TimerEvent& event);
static void control_update_timer_cb(const ros::TimerEvent& event);
static void log_report_timer_cb(const ros::TimerEvent& event);

/* Main function / 主函数 -----------------------------------------------------*/

/**
 * @brief 控制器调度节点入口：加载参数、初始化控制器和订阅、进入主循环。
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    update_rate = pnh.param<double>("update_rate", 50.0);
    runtime_check_rate = pnh.param<double>("runtime_check_rate", 20.0);
    command_timeout = pnh.param<double>("command_timeout", 0.3);
    vehicle_status_timeout = pnh.param<double>("vehicle_status_timeout", 2.0);
    local_position_timeout = pnh.param<double>("local_position_timeout", 2.0);
    log_report_rate = pnh.param<double>("log_report_rate", 1.0);
    mavros_ns = pnh.param<std::string>("mavros_ns", "/mavros");

    if (update_rate <= 0.0) {
        ROS_WARN("[Controller_node] update_rate must be positive, reset to 50.0");
        update_rate = 50.0;
    }
    if (runtime_check_rate <= 0.0) {
        ROS_WARN("[Controller_node] runtime_check_rate must be positive, reset to 20.0");
        runtime_check_rate = 20.0;
    }
    if (command_timeout <= 0.0) {
        ROS_WARN("[Controller_node] command_timeout must be positive, reset to 0.3");
        command_timeout = 0.3;
    }
    if (vehicle_status_timeout <= 0.0) {
        ROS_WARN("[Controller_node] vehicle_status_timeout must be positive, reset to 2.0");
        vehicle_status_timeout = 2.0;
    }
    if (local_position_timeout <= 0.0) {
        ROS_WARN("[Controller_node] local_position_timeout must be positive, reset to 2.0");
        local_position_timeout = 2.0;
    }
    if (log_report_rate <= 0.0) {
        ROS_WARN("[Controller_node] log_report_rate must be positive, reset to 1.0");
        log_report_rate = 1.0;
    }

    ros::NodeHandle pid_nh("~/pid_position_controller");
    pid_position_controller.loadParams(pid_nh);

    double tracking_timeout = pnh.param<double>("rl/tracking_output_timeout", 0.15);
    double landing_timeout = pnh.param<double>("rl/landing_output_timeout", 0.15);
    rl_tracking.setTimeout(tracking_timeout);
    rl_landing.setTimeout(landing_timeout);

    tracking_mix.rl_start_distance = pnh.param<double>("tracking_mix/rl_start_distance", 5.0);
    tracking_mix.rl_full_distance = pnh.param<double>("tracking_mix/rl_full_distance", 1.0);
    tracking_mix.rl_alpha_max = pnh.param<double>("tracking_mix/rl_alpha_max", 0.3);
    tracking_mix.min_target_confidence = pnh.param<double>("tracking_mix/min_target_confidence", 0.7);

    landing_mix.rl_alpha_max = pnh.param<double>("landing_mix/rl_alpha_max", 0.3);
    landing_mix.min_target_confidence = pnh.param<double>("landing_mix/min_target_confidence", 0.8);

    // ── 话题与服务名称 ──────────────────────────────────────────────────
    const std::string control_request_topic = "/uav/control/request";         // 任务层控制请求 (任务节点发布)
    const std::string vehicle_status_topic = "/uav/vehicle/status";           // 车辆连接状态 (connect_manager 发布)
    const std::string target_state_topic = "/uav/target_state";               // 目标状态 (target_estimator 发布)
    const std::string rl_tracking_topic = "/uav/rl/tracking_output";          // RL 追踪速度输出
    const std::string rl_landing_topic = "/uav/rl/landing_output";            // RL 降落速度输出
    const std::string candidate_velocity_topic = "/uav/control/candidate_velocity";  // 候选速度 (velocity_manager 消费)
    const std::string controller_feedback_topic = "/uav/control/controller_feedback"; // 控制器反馈状态
    const std::string log_report_topic = "/uav/log_reports/controller";       // 节点日志报告 (logger_node 消费)

    // 输入回调只缓存控制请求、飞行状态、目标状态和 RL 输出，主循环统一调度控制器。
    control_request_sub = nh.subscribe(control_request_topic, 10, control_request_cb);
    vehicle_status_sub = nh.subscribe(vehicle_status_topic, 10, vehicle_status_cb);
    pose_sub = nh.subscribe(mavros_ns + "/local_position/pose", 10, pose_cb);
    vel_sub = nh.subscribe(mavros_ns + "/local_position/velocity_local", 10, vel_cb);
    target_state_sub = nh.subscribe(target_state_topic, 10, target_state_cb);
    rl_tracking_sub = nh.subscribe(rl_tracking_topic, 10, rl_tracking_cb);
    rl_landing_sub = nh.subscribe(rl_landing_topic, 10, rl_landing_cb);

    candidate_vel_pub = nh.advertise<lite_quad_uav::VelocityCommand>(candidate_velocity_topic, 10);
    feedback_pub = nh.advertise<lite_quad_uav::ControllerFeedback>(controller_feedback_topic, 10);
    log_report_pub = nh.advertise<lite_quad_uav::ControllerLogReport>(log_report_topic, 10);
    ros::Timer runtime_check_timer = nh.createTimer(ros::Duration(1.0 / runtime_check_rate), runtime_check_timer_cb);
    ros::Timer control_update_timer = nh.createTimer(ros::Duration(1.0 / update_rate), control_update_timer_cb);
    ros::Timer log_report_timer = nh.createTimer(ros::Duration(1.0 / log_report_rate), log_report_timer_cb);

    ROS_INFO("[Controller_node] Initialized, rate = %.0f, runtime_check_rate = %.1f, command_timeout = %.2f, mavros_ns = %s",
             update_rate, runtime_check_rate, command_timeout, mavros_ns.c_str());

    ros::spin();
    return 0;
}

/* Private function implementations / 私有函数实现 ----------------------------*/

static const char* node_state_to_string(ControllerNodeState state) {
    switch (state) {
    case ControllerNodeState::INITIALIZING:   return "INITIALIZING";
    case ControllerNodeState::NORMAL:         return "NORMAL";
    case ControllerNodeState::REINITIALIZING: return "REINITIALIZING";
    }
    return "UNKNOWN";
}

static void transition_to(ControllerNodeState next_state) {
    if (node_state == next_state) {
        return;
    }

    ROS_INFO("[Controller_node] node_state: %s -> %s",
             node_state_to_string(node_state), node_state_to_string(next_state));
    node_state = next_state;
}

static void update_runtime_check_flags(const ros::Time& now) {
    runtime_flags.vehicle_status_fresh =
        vehicle_status_received && (now - last_vehicle_status_time).toSec() <= vehicle_status_timeout;
    runtime_flags.vehicle_connected = runtime_flags.vehicle_status_fresh && vehicle_status.connected;
    runtime_flags.local_position_fresh =
        local_position_received && (now - last_local_position_time).toSec() <= local_position_timeout;
    runtime_flags.reliable_vehicle_connection = runtime_flags.vehicle_connected;
    runtime_flags.command_valid = command_received && latest_command.valid;
    runtime_flags.command_fresh =
        command_received && latest_command.valid && !command_timeout_reached(now);

    vehicle_state.valid = runtime_flags.local_position_fresh;
}

static void update_node_state(const ros::Time& /*now*/) {
    switch (node_state) {
    case ControllerNodeState::INITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_to(ControllerNodeState::NORMAL);
        }
        break;

    case ControllerNodeState::NORMAL:
        if (!runtime_flags.reliable_vehicle_connection) {
            transition_to(ControllerNodeState::REINITIALIZING);
        }
        break;

    case ControllerNodeState::REINITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_to(ControllerNodeState::NORMAL);
        }
        break;
    }
}

static void refresh_controller_context(const ros::Time& now) {
    update_runtime_check_flags(now);
    update_node_state(now);
}

/**
 * @brief 控制请求回调：将任务层 ControlRequest 转换为控制器内部 ControllerCommand。
 * @param[in] msg 控制请求消息。
 */
static void control_request_cb(const lite_quad_uav::ControlRequest::ConstPtr& msg) {
    latest_command = control_request_to_command(*msg);
    last_command_time = ros::Time::now();
    command_received = true;
}

static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg) {
    vehicle_status = *msg;
    last_vehicle_status_time = ros::Time::now();
    vehicle_status_received = true;
}

/**
 * @brief 位置回调，更新飞行器位置和偏航角。
 * @param[in] msg 本地位置消息。
 */
static void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    vehicle_state.x = msg->pose.position.x;
    vehicle_state.y = msg->pose.position.y;
    vehicle_state.z = msg->pose.position.z;
    vehicle_state.yaw = tf::getYaw(msg->pose.orientation);
    last_local_position_time = ros::Time::now();
    local_position_received = true;
}

/**
 * @brief 速度回调，更新飞行器线速度。
 * @param[in] msg 本地速度消息。
 */
static void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vehicle_state.vx = msg->twist.linear.x;
    vehicle_state.vy = msg->twist.linear.y;
    vehicle_state.vz = msg->twist.linear.z;
}

/**
 * @brief 目标状态回调，缓存最新目标信息。
 * @param[in] msg 目标状态消息。
 */
static void target_state_cb(const lite_quad_uav::TargetState::ConstPtr& msg) {
    target_state = *msg;
}

/**
 * @brief RL 追踪输出回调，更新追踪缓存。
 * @param[in] msg RL 速度命令。
 */
static void rl_tracking_cb(const lite_quad_uav::VelocityCommand::ConstPtr& msg) {
    rl_tracking.update(*msg);
}

/**
 * @brief RL 降落输出回调，更新降落缓存。
 * @param[in] msg RL 速度命令。
 */
static void rl_landing_cb(const lite_quad_uav::VelocityCommand::ConstPtr& msg) {
    rl_landing.update(*msg);
}

/**
 * @brief 主控制器调度：根据 controller_type 和 mix_mode 选择 PID / RL / 混合模式。
 * @param[in] now 当前时间。
 */
static void update_controller(const ros::Time& now) {
    refresh_controller_context(now);

    if (node_state != ControllerNodeState::NORMAL) {
        reset_pid_state();
        publish_invalid(std::string("Waiting for reliable vehicle connection: ") + node_state_to_string(node_state), now);
        return;
    }

    // 前置条件检查
    if (!runtime_flags.local_position_fresh) {
        reset_pid_state();
        publish_invalid("local_position/pose timeout", now);
        return;
    }

    if (!runtime_flags.command_valid) {
        reset_pid_state();
        publish_invalid("control command invalid", now);
        return;
    }

    if (!runtime_flags.command_fresh) {
        reset_pid_state();
        publish_invalid("control command timeout", now);
        return;
    }

    ControllerOutput output;

    switch (latest_command.controller_type) {
    case ControllerCommand::CONTROLLER_IDLE:
        reset_pid_state();
        output.valid = false;
        output.controller_name = "idle";
        output.message = "Controller idle";
        break;

    case ControllerCommand::CONTROLLER_PID_POSITION:
        reset_pid_if_command_context_changed();
        output = run_pid_position(now);
        break;

    case ControllerCommand::CONTROLLER_RL_TRACKING:
        if (latest_command.mix_mode == ControllerCommand::MIX_PID_RL_TRACKING || latest_command.mix_mode == ControllerCommand::MIX_AUTO) {
            reset_pid_if_command_context_changed();
            output = run_pid_rl_tracking_mix(now);
        } else if (latest_command.mix_mode == ControllerCommand::MIX_PID_ONLY) {
            reset_pid_if_command_context_changed();
            output = run_pid_position(now);
        } else {
            reset_pid_state();
            output = run_rl_tracking(now);
        }
        break;

    case ControllerCommand::CONTROLLER_RL_LANDING:
        if (latest_command.mix_mode == ControllerCommand::MIX_PID_RL_LANDING || latest_command.mix_mode == ControllerCommand::MIX_AUTO) {
            reset_pid_if_command_context_changed();
            output = run_pid_rl_landing_mix(now);
        } else if (latest_command.mix_mode == ControllerCommand::MIX_PID_ONLY) {
            reset_pid_if_command_context_changed();
            output = run_pid_position(now);
        } else {
            reset_pid_state();
            output = run_rl_landing(now);
        }
        break;

    default:
        reset_pid_state();
        output.valid = false;
        output.controller_name = "unknown";
        output.message = "Unsupported controller type";
        break;
    }

    apply_command_velocity_limits(output);
    publish_velocity(output, now);
    publish_feedback(output, now);
}

/**
 * @brief 检查控制命令是否超时。
 * @param[in] now 当前时间。
 * @return true=已超时。
 */
static bool command_timeout_reached(const ros::Time& now) {
    double age = (now - last_command_time).toSec();
    double timeout = (latest_command.timeout_sec > 0.0) ? latest_command.timeout_sec : command_timeout;
    return age > timeout;
}

/**
 * @brief 将任务层 ControlRequest 转换为控制器内部 ControllerCommand。
 * @param[in] request 任务层控制请求。
 * @return PID/RL 调度使用的内部控制命令。
 *
 * @note reference_velocity_ff 本轮暂不接入 PID，只保留在 ControlRequest 中作为后续前馈扩展。
 */
static ControllerCommand control_request_to_command(const lite_quad_uav::ControlRequest& request) {
    ControllerCommand command;
    command.header = request.header;
    command.source_task = request.source_task;
    command.source_action = request.source_action;
    command.target_x = request.reference_position.x;
    command.target_y = request.reference_position.y;
    command.target_z = request.reference_position.z;
    command.target_yaw = request.reference_yaw;
    command.use_yaw = request.max_yaw_rate > 0.0;
    command.max_xy_speed = request.max_xy_speed;
    command.max_z_up_speed = request.max_z_up_speed;
    command.max_z_down_speed = request.max_z_down_speed;
    command.max_yaw_rate = request.max_yaw_rate;
    command.timeout_sec = 0.0;

    if (!request.valid || request.mode == "IDLE") {
        command.controller_type = ControllerCommand::CONTROLLER_IDLE;
        command.mix_mode = ControllerCommand::MIX_NONE;
        command.valid = true;
        command.message = "Idle control request";
        return command;
    }

    if (request.mode == "REFERENCE_TRACKING" || request.mode == "HOLD") {
        command.controller_type = ControllerCommand::CONTROLLER_PID_POSITION;
        command.mix_mode = ControllerCommand::MIX_PID_ONLY;
        command.valid = true;
        command.message = request.mode;
        return command;
    }

    command.controller_type = ControllerCommand::CONTROLLER_IDLE;
    command.mix_mode = ControllerCommand::MIX_NONE;
    command.valid = false;
    command.message = "Unsupported ControlRequest mode: " + request.mode;
    return command;
}

/**
 * @brief 完全重置 PID 控制器状态和上下文。
 */
static void reset_pid_state() {
    pid_position_controller.reset();
    last_pid_update_time = ros::Time(0);
    pid_context_initialized = false;
    last_pid_source_task = "";
    last_pid_source_action = "";
    last_control_type = -1;
    last_pid_mix_mode = -1;
}

/**
 * @brief 若控制命令的 source_task/source_action/controller_type/mix_mode 任一变化，重置 PID 状态。
 *
 * @note 切换任务或控制模式时必须重置，否则 PID 积分和微分状态残留会导致输出跳变。
 */
static void reset_pid_if_command_context_changed() {
    bool changed = !pid_context_initialized ||
                   last_pid_source_task != latest_command.source_task ||
                   last_pid_source_action != latest_command.source_action ||
                   last_control_type != static_cast<int>(latest_command.controller_type) ||
                   last_pid_mix_mode != static_cast<int>(latest_command.mix_mode);

    if (changed) {
        pid_position_controller.reset();
        last_pid_update_time = ros::Time(0);
        last_pid_source_task = latest_command.source_task;
        last_pid_source_action = latest_command.source_action;
        last_control_type = static_cast<int>(latest_command.controller_type);
        last_pid_mix_mode = static_cast<int>(latest_command.mix_mode);
        pid_context_initialized = true;
    }
}

/**
 * @brief 计算本次 PID 更新的时间步长 dt。
 * @param[in] now 当前时间。
 * @return 时间步长 (s)，异常时回退到 1/update_rate。
 */
static double get_pid_dt(const ros::Time& now) {
    double default_dt = 1.0 / update_rate;
    double dt = default_dt;

    if (last_pid_update_time != ros::Time(0)) {
        dt = (now - last_pid_update_time).toSec();
        if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
            dt = default_dt;
        }
    }

    last_pid_update_time = now;
    return dt;
}

/**
 * @brief 使用指定控制命令执行 PID 位置控制。
 * @param[in] command 控制命令。
 * @param[in] now 当前时间。
 * @return 控制器输出。
 */
static ControllerOutput update_pid_position_with_command(const ControllerCommand& command, const ros::Time& now) {
    double dt = get_pid_dt(now);
    return pid_position_controller.update(vehicle_state, command, dt);
}

/**
 * @brief 纯 PID 位置控制模式。
 * @param[in] now 当前时间。
 * @return 控制器输出。
 */
static ControllerOutput run_pid_position(const ros::Time& now) {
    return update_pid_position_with_command(latest_command, now);
}

/**
 * @brief 纯 RL 追踪模式：直接转发 RL 网络输出。
 * @param[in] now 当前时间。
 * @return 控制器输出。RL 无效时返回 invalid。
 */
static ControllerOutput run_rl_tracking(const ros::Time& now) {
    ControllerOutput output;
    output.controller_name = "rl_tracking";

    if (!rl_tracking.isValid(now)) {
        output.valid = false;
        output.message = "RL tracking output invalid or timeout";
        return output;
    }

    lite_quad_uav::VelocityCommand rl_cmd = rl_tracking.latest();
    output.vx = rl_cmd.vx;
    output.vy = rl_cmd.vy;
    output.vz = rl_cmd.vz;
    output.yaw_rate = rl_cmd.yaw_rate;
    output.valid = true;
    output.message = "RL tracking direct";
    return output;
}

/**
 * @brief 纯 RL 降落模式：直接转发 RL 网络输出。
 * @param[in] now 当前时间。
 * @return 控制器输出。RL 无效时返回 invalid。
 */
static ControllerOutput run_rl_landing(const ros::Time& now) {
    ControllerOutput output;
    output.controller_name = "rl_landing";

    if (!rl_landing.isValid(now)) {
        output.valid = false;
        output.message = "RL landing output invalid or timeout";
        return output;
    }

    lite_quad_uav::VelocityCommand rl_cmd = rl_landing.latest();
    output.vx = rl_cmd.vx;
    output.vy = rl_cmd.vy;
    output.vz = rl_cmd.vz;
    output.yaw_rate = rl_cmd.yaw_rate;
    output.valid = true;
    output.message = "RL landing direct";
    return output;
}

/**
 * @brief PID+RL 追踪混合模式：PID 输出加上 alpha×RL 输出。
 * @param[in] now 当前时间。
 * @return 混合控制器输出。
 *
 * @note 混合方式为加法 (PID + alpha×RL)，非加权平均，
 *       输出可能超过单一控制器单独输出的限幅范围。
 */
static ControllerOutput run_pid_rl_tracking_mix(const ros::Time& now) {
    ControllerOutput output;
    output.controller_name = "pid_rl_tracking_mix";

    bool rl_valid = rl_tracking.isValid(now);
    bool target_valid = target_state.valid && target_state.confidence >= tracking_mix.min_target_confidence;

    // 首版假设 TargetState 已转换到 MAVROS local ENU，target 有效时用它覆盖 PID 目标。
    ControllerCommand pid_command = latest_command;
    if (target_valid) {
        pid_command.target_x = target_state.target_x;
        pid_command.target_y = target_state.target_y;
        pid_command.target_z = target_state.target_z;
        pid_command.target_yaw = target_state.target_yaw;
        pid_command.use_yaw = true;
    }

    ControllerOutput pid_output = update_pid_position_with_command(pid_command, now);

    // 计算目标距离
    double distance = 0.0;
    if (target_valid) {
        double dx = target_state.target_x - vehicle_state.x;
        double dy = target_state.target_y - vehicle_state.y;
        double dz = target_state.target_z - vehicle_state.z;
        distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double alpha = compute_tracking_mix_weight(distance, target_valid ? target_state.confidence : 0.0, rl_valid);

    if (rl_valid && alpha > 0.0) {
        lite_quad_uav::VelocityCommand rl_cmd = rl_tracking.latest();
        output.vx = pid_output.vx + alpha * rl_cmd.vx;
        output.vy = pid_output.vy + alpha * rl_cmd.vy;
        output.vz = pid_output.vz + alpha * rl_cmd.vz;
        output.yaw_rate = pid_output.yaw_rate + alpha * rl_cmd.yaw_rate;
        output.message = "PID+RL tracking mix, alpha=" + std::to_string(alpha);
    } else {
        output.vx = pid_output.vx;
        output.vy = pid_output.vy;
        output.vz = pid_output.vz;
        output.yaw_rate = pid_output.yaw_rate;
        output.fallback_active = true;
        output.message = "Fallback to PID (RL invalid or alpha=0)";
    }

    output.valid = pid_output.valid;
    output.position_error_norm = pid_output.position_error_norm;
    output.target_distance = distance;
    output.rl_weight = alpha;
    output.converged = pid_output.converged;
    output.saturated = pid_output.saturated;

    return output;
}

/**
 * @brief PID+RL 降落混合模式：PID 输出加上 beta×RL 输出。
 * @param[in] now 当前时间。
 * @return 混合控制器输出。
 *
 * @note 混合方式为加法 (PID + beta×RL)，非加权平均，
 *       输出可能超过单一控制器单独输出的限幅范围。
 */
static ControllerOutput run_pid_rl_landing_mix(const ros::Time& now) {
    ControllerOutput output;
    output.controller_name = "pid_rl_landing_mix";

    bool rl_valid = rl_landing.isValid(now);
    bool target_valid = target_state.valid && target_state.confidence >= landing_mix.min_target_confidence;

    // 首版用 TargetState 的 X/Y/Z 作为 PID baseline 目标；真实降落策略后续再细化。
    ControllerCommand pid_command = latest_command;
    if (target_valid) {
        pid_command.target_x = target_state.target_x;
        pid_command.target_y = target_state.target_y;
        pid_command.target_z = target_state.target_z;
        pid_command.target_yaw = target_state.target_yaw;
        pid_command.use_yaw = true;
    }

    ControllerOutput pid_output = update_pid_position_with_command(pid_command, now);

    double distance = 0.0;
    if (target_valid) {
        double dx = target_state.target_x - vehicle_state.x;
        double dy = target_state.target_y - vehicle_state.y;
        double dz = target_state.target_z - vehicle_state.z;
        distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double beta = compute_landing_mix_weight(target_valid ? target_state.confidence : 0.0, rl_valid);

    if (rl_valid && beta > 0.0) {
        lite_quad_uav::VelocityCommand rl_cmd = rl_landing.latest();
        output.vx = pid_output.vx + beta * rl_cmd.vx;
        output.vy = pid_output.vy + beta * rl_cmd.vy;
        output.vz = pid_output.vz + beta * rl_cmd.vz;
        output.yaw_rate = pid_output.yaw_rate + beta * rl_cmd.yaw_rate;
        output.message = "PID+RL landing mix, beta=" + std::to_string(beta);
    } else {
        output.vx = pid_output.vx;
        output.vy = pid_output.vy;
        output.vz = pid_output.vz;
        output.yaw_rate = pid_output.yaw_rate;
        output.fallback_active = true;
        output.message = "Fallback to PID baseline (RL invalid or beta=0)";
    }

    output.valid = pid_output.valid;
    output.position_error_norm = pid_output.position_error_norm;
    output.target_distance = distance;
    output.rl_weight = beta;
    output.converged = pid_output.converged;
    output.saturated = pid_output.saturated;

    return output;
}

/**
 * @brief 根据 RL 有效性、目标置信度和距离计算追踪混合权重 alpha。
 * @param[in] distance 目标距离 (m)。
 * @param[in] confidence 目标置信度 [0,1]。
 * @param[in] rl_valid RL 输出是否有效。
 * @return 混合权重 alpha [0, rl_alpha_max]。
 *
 * @note alpha 在 rl_full_distance 处最大，线性递减到 rl_start_distance 处为 0。
 */
static double compute_tracking_mix_weight(double distance, double confidence, bool rl_valid) {
    if (!rl_valid) return 0.0;
    if (!target_state.valid) return 0.0;
    if (confidence < tracking_mix.min_target_confidence) return 0.0;

    if (distance >= tracking_mix.rl_start_distance) return 0.0;
    if (distance <= tracking_mix.rl_full_distance) return tracking_mix.rl_alpha_max;

    double ratio = (tracking_mix.rl_start_distance - distance) / (tracking_mix.rl_start_distance - tracking_mix.rl_full_distance);
    return tracking_mix.rl_alpha_max * ratio;
}

/**
 * @brief 计算降落混合权重 beta：RL 有效且置信度足够时固定为 rl_alpha_max。
 * @param[in] confidence 目标置信度。
 * @param[in] rl_valid RL 输出是否有效。
 * @return 混合权重 beta [0, rl_alpha_max]。
 */
static double compute_landing_mix_weight(double confidence, bool rl_valid) {
    if (!rl_valid) return 0.0;
    if (!target_state.valid) return 0.0;
    if (confidence < landing_mix.min_target_confidence) return 0.0;

    return landing_mix.rl_alpha_max;
}

/**
 * @brief 对所有控制器输出执行命令级候选速度限幅。
 * @param[in,out] output 控制器输出。
 *
 * @note PID 内部已有一次限幅；这里作为统一出口保护，覆盖 RL 直出和 PID+RL 混合输出。
 */
static void apply_command_velocity_limits(ControllerOutput& output) {
    if (!output.valid) return;

    double max_xy_speed = std::max(0.0, latest_command.max_xy_speed);
    double max_z_up_speed = std::max(0.0, latest_command.max_z_up_speed);
    double max_z_down_speed = std::max(0.0, latest_command.max_z_down_speed);
    double max_yaw_rate = std::max(0.0, latest_command.max_yaw_rate);

    bool saturated = false;
    double speed_xy = std::sqrt(output.vx * output.vx + output.vy * output.vy);
    if (speed_xy > max_xy_speed && speed_xy > 0.0) {
        double scale = max_xy_speed / speed_xy;
        output.vx *= scale;
        output.vy *= scale;
        saturated = true;
    }

    if (output.vz > max_z_up_speed) {
        output.vz = max_z_up_speed;
        saturated = true;
    } else if (output.vz < -max_z_down_speed) {
        output.vz = -max_z_down_speed;
        saturated = true;
    }

    if (output.yaw_rate > max_yaw_rate) {
        output.yaw_rate = max_yaw_rate;
        saturated = true;
    } else if (output.yaw_rate < -max_yaw_rate) {
        output.yaw_rate = -max_yaw_rate;
        saturated = true;
    }

    output.saturated = output.saturated || saturated;
}

/**
 * @brief 将 ControllerOutput 转换为 VelocityCommand 并发布到候选速度话题。
 * @param[in] output 控制器输出。
 * @param[in] now 当前时间。
 */
static void publish_velocity(const ControllerOutput& output, const ros::Time& now) {
    lite_quad_uav::VelocityCommand cmd;
    cmd.header.stamp = now;
    cmd.source = output.controller_name;
    cmd.controller_name = output.controller_name;
    cmd.vx = output.vx;
    cmd.vy = output.vy;
    cmd.vz = output.vz;
    cmd.yaw_rate = output.yaw_rate;
    cmd.valid = output.valid;
    cmd.message = output.message;
    candidate_vel_pub.publish(cmd);
}

/**
 * @brief 发布控制器反馈状态。
 * @param[in] output 控制器输出。
 * @param[in] now 当前时间。
 */
static void publish_feedback(const ControllerOutput& output, const ros::Time& now) {
    lite_quad_uav::ControllerFeedback fb;
    fb.header.stamp = now;
    fb.controller_name = output.controller_name;

    if (node_state != ControllerNodeState::NORMAL) fb.state = node_state_to_string(node_state);
    else if (!vehicle_state.valid)  fb.state = "NO_VEHICLE";
    else if (!command_received)     fb.state = "NO_COMMAND";
    else if (!latest_command.valid) fb.state = "COMMAND_INVALID";
    else if (output.valid)          fb.state = "ACTIVE";
    else                            fb.state = "INVALID";

    fb.position_error_norm = output.position_error_norm;
    fb.converged = output.converged;
    fb.saturated = output.saturated;
    fb.message = output.message;

    fb.source_task = latest_command.source_task;
    fb.source_action = latest_command.source_action;
    fb.controller_type = latest_command.controller_type;
    fb.mix_mode = latest_command.mix_mode;
    fb.target_distance = output.target_distance;
    fb.rl_weight = output.rl_weight;

    fb.command_valid = runtime_flags.command_valid;
    fb.vehicle_state_valid = vehicle_state.valid;
    fb.target_state_valid = target_state.valid;
    fb.rl_tracking_valid = rl_tracking.isValid(now);
    fb.rl_landing_valid = rl_landing.isValid(now);
    fb.fallback_active = output.fallback_active;

    feedback_pub.publish(fb);
    last_controller_output = output;
    controller_output_received = true;
    last_feedback = fb;
    feedback_received = true;
}

/**
 * @brief 发布无效速度和反馈（车辆状态/命令/超时异常时调用）。
 * @param[in] reason 无效原因。
 * @param[in] now 当前时间。
 */
static void publish_invalid(const std::string& reason, const ros::Time& now) {
    lite_quad_uav::VelocityCommand cmd;
    cmd.header.stamp = now;
    cmd.source = "controller_invalid";
    cmd.controller_name = "controller_invalid";
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.vz = 0.0;
    cmd.yaw_rate = 0.0;
    cmd.valid = false;
    cmd.message = reason;
    candidate_vel_pub.publish(cmd);

    lite_quad_uav::ControllerFeedback fb;
    fb.header.stamp = now;
    fb.controller_name = "controller_invalid";
    fb.state = (node_state != ControllerNodeState::NORMAL) ? node_state_to_string(node_state) : "INVALID";
    fb.position_error_norm = 0.0;
    fb.converged = false;
    fb.saturated = false;
    fb.message = reason;

    fb.source_task = latest_command.source_task;
    fb.source_action = latest_command.source_action;
    fb.controller_type = latest_command.controller_type;
    fb.mix_mode = latest_command.mix_mode;
    fb.target_distance = 0.0;
    fb.rl_weight = 0.0;

    fb.command_valid = runtime_flags.command_valid;
    fb.vehicle_state_valid = vehicle_state.valid;
    fb.target_state_valid = target_state.valid;
    fb.rl_tracking_valid = rl_tracking.isValid(now);
    fb.rl_landing_valid = rl_landing.isValid(now);
    fb.fallback_active = false;

    feedback_pub.publish(fb);
    ControllerOutput output;
    output.valid = false;
    output.controller_name = "controller_invalid";
    output.message = reason;
    last_controller_output = output;
    controller_output_received = true;
    last_feedback = fb;
    feedback_received = true;
}

static void runtime_check_timer_cb(const ros::TimerEvent& /*event*/) {
    update_runtime_check_flags(ros::Time::now());
}

static void control_update_timer_cb(const ros::TimerEvent& /*event*/) {
    update_controller(ros::Time::now());
}

/**
 * @brief 周期发布控制器日志报告，供 logger_node 集中展示。
 * @param[in] event 定时器事件。
 */
static void log_report_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    refresh_controller_context(now);

    lite_quad_uav::ControllerLogReport report;
    report.header.stamp = now;
    report.controller_name = feedback_received ? last_feedback.controller_name : "";
    report.node_state = node_state_to_string(node_state);
    report.state = feedback_received ? last_feedback.state : "NO_FEEDBACK";
    report.controller_type = command_received ? latest_command.controller_type : 0;
    report.mix_mode = command_received ? latest_command.mix_mode : 0;
    report.source_task = command_received ? latest_command.source_task : "";
    report.source_action = command_received ? latest_command.source_action : "";

    report.command_received = command_received;
    report.command_valid = runtime_flags.command_valid;
    report.command_fresh = runtime_flags.command_fresh;
    report.vehicle_state_valid = vehicle_state.valid;
    report.vehicle_status_fresh = runtime_flags.vehicle_status_fresh;
    report.local_position_fresh = runtime_flags.local_position_fresh;
    report.reliable_vehicle_connection = runtime_flags.reliable_vehicle_connection;
    report.target_state_valid = target_state.valid;
    report.rl_tracking_valid = rl_tracking.isValid(now);
    report.rl_landing_valid = rl_landing.isValid(now);

    report.output_valid = controller_output_received && last_controller_output.valid;
    report.output_vx = controller_output_received ? last_controller_output.vx : 0.0;
    report.output_vy = controller_output_received ? last_controller_output.vy : 0.0;
    report.output_vz = controller_output_received ? last_controller_output.vz : 0.0;
    report.output_yaw_rate = controller_output_received ? last_controller_output.yaw_rate : 0.0;

    report.position_error_norm = feedback_received ? last_feedback.position_error_norm : 0.0;
    report.target_distance = feedback_received ? last_feedback.target_distance : 0.0;
    report.rl_weight = feedback_received ? last_feedback.rl_weight : 0.0;
    report.converged = feedback_received && last_feedback.converged;
    report.saturated = feedback_received && last_feedback.saturated;
    report.fallback_active = feedback_received && last_feedback.fallback_active;
    report.message = feedback_received ? last_feedback.message : "No controller feedback published yet";

    log_report_pub.publish(report);
}
