/**
 * @file square_patrol_task_node.cpp
 * @brief 方形巡航任务节点：维护任务状态机并通过 Action 生成 ControlRequest。
 *
 * 遵守统一任务接口：
 *   - start service:  /uav/tasks/square_patrol/start   (StartTask)
 *   - control service: /uav/tasks/square_patrol/control (ControlTask)
 *   - status topic:   /uav/tasks/square_patrol/status  (TaskStatus)
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <lite_quad_uav/ControlRequest.h>
#include <lite_quad_uav/ControlTask.h>
#include <lite_quad_uav/ControllerFeedback.h>
#include <lite_quad_uav/StartTask.h>
#include <lite_quad_uav/TaskStatus.h>
#include <lite_quad_uav/TaskNodeLogReport.h>
#include <lite_quad_uav/VehicleCommand.h>
#include <lite_quad_uav/VehicleStatus.h>
#include <lite_quad_uav/tasks/actions.hpp>

/* Private typedef / 私有类型定义 --------------------------------------------*/

enum SquarePatrolTaskState {
    TASK_IDLE = 0,        // 空闲
    TASK_PRECHECK,        // 预检查（等待飞行器状态有效）
    TASK_MODE_REQUEST,    // 请求 OFFBOARD 模式
    TASK_MODE_WAIT_CONFIRM,// 等待 OFFBOARD 确认
    TASK_ARM_REQUEST,     // 请求解锁
    TASK_ARM_WAIT_CONFIRM,// 等待解锁确认
    TASK_TAKEOFF,         // 起飞中
    TASK_PATROL,          // 巡航中
    TASK_HOLD,            // 悬停（当前未使用）
    TASK_PAUSED,          // 暂停中
    TASK_CANCELED,        // 已取消
    TASK_FAILED           // 已失败
};

enum class SquarePatrolNodeState {
    INITIALIZING = 0,
    NORMAL,
    REINITIALIZING
};

struct SquarePatrolRuntimeFlags {
    bool local_position_fresh = false;
    bool vehicle_status_fresh = false;
    bool vehicle_connected = false;
    bool reliable_vehicle_connection = false;
    bool vehicle_ready = false;
    bool vehicle_armed = false;
};

/* Private variables / 私有变量 ----------------------------------------------*/

static SquarePatrolTaskState task_progress = TASK_IDLE;
static SquarePatrolTaskState paused_from_state = TASK_IDLE;
static SquarePatrolNodeState node_state = SquarePatrolNodeState::INITIALIZING;

static std::unique_ptr<ActionBase> active_action;
static std::unique_ptr<HoldAction> pause_hold_action;
static std::string active_action_name;
static std::string task_state = "IDLE";
static std::string task_message;

static std::string mavros_ns = "/mavros";
static double square_side_length = 10.0;
static double flight_altitude = 3.0;
static double takeoff_height = 3.0;
static double waypoint_accept_radius = 0.4;
static double waypoint_hold_time = 0.0;
static double max_xy_speed = 10.0;
static double max_z_up_speed = 0.6;
static double max_z_down_speed = 0.6;
static double max_yaw_rate = 0.0;
static double update_rate = 20.0;
static double runtime_check_rate = 20.0;
static double local_position_timeout = 2.0;
static double vehicle_status_timeout = 2.0;
static double mode_confirm_timeout = 10.0;
static double arm_confirm_timeout = 10.0;
static double log_report_rate = 1.0;
static bool auto_start = true;

static VehicleState vehicle_state;
static ControllerFeedbackData controller_feedback;
static bool local_position_received = false;
static bool controller_feedback_received = false;
static std::vector<Waypoint> waypoints;
static bool patrol_origin_locked = false;
static double patrol_origin_x = 0.0;
static double patrol_origin_y = 0.0;
static ros::Time state_enter_time;
static ros::Time last_local_position_time;
static ros::Time last_controller_feedback_time;

static lite_quad_uav::VehicleStatus vehicle_status;
static bool vehicle_status_received = false;
static ros::Time last_vehicle_status_time;
static SquarePatrolRuntimeFlags runtime_flags;

static ros::Subscriber local_pos_sub;
static ros::Subscriber vehicle_status_sub;
static ros::Subscriber controller_feedback_sub;
static ros::Publisher control_request_pub;
static ros::Publisher status_pub;
static ros::Publisher log_report_pub;
static ros::ServiceServer start_srv;
static ros::ServiceServer control_srv;
static ros::ServiceClient vehicle_command_client;
static ros::Timer update_timer;
static ros::Timer runtime_check_timer;
static ros::Timer log_report_timer;

/* Private function prototypes / 私有函数声明 --------------------------------*/

static void load_params(ros::NodeHandle& pnh);
static void generate_waypoints(double origin_x, double origin_y);
static const char* node_state_to_string(SquarePatrolNodeState state);
static const char* task_progress_to_string(SquarePatrolTaskState progress);
static void transition_node_to(SquarePatrolNodeState next_state);
static bool task_is_busy();
static bool task_is_paused();
static ActionInput make_action_input(const ros::Time& now);
static void transition_to(SquarePatrolTaskState next_state);
static void activate_task(const ros::Time& now, const std::string& reason);
static void lock_patrol_origin();
static void enter_takeoff(const ros::Time& now);
static void enter_patrol(const ros::Time& now);
static void enter_paused(const ros::Time& now);
static lite_quad_uav::ControlRequest make_idle_request(const ros::Time& now);
static void publish_output(const ActionOutput& output);
static bool local_position_fresh(const ros::Time& now);
static bool vehicle_status_fresh(const ros::Time& now);
static void update_runtime_check_flags(const ros::Time& now);
static void update_node_state(const ros::Time& now);
static void refresh_task_context(const ros::Time& now);
static bool precheck_ready(const ros::Time& now, std::string& reason);
static bool flight_runtime_ready(const ros::Time& now, std::string& reason);
static bool vehicle_mode_is(const std::string& mode);
static bool request_vehicle_mode(const std::string& mode, std::string& message);
static bool request_arm(std::string& message);
static void fail_task(const std::string& reason);
static void cancel_task(const std::string& reason);
static void publish_status(const ros::Time& now);
static std::string normalize_command(const std::string& command);
static bool handle_pause(lite_quad_uav::ControlTask::Response& res);
static bool handle_resume(lite_quad_uav::ControlTask::Response& res);
static bool handle_cancel(lite_quad_uav::ControlTask::Response& res);
static void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg);
static void controller_feedback_cb(const lite_quad_uav::ControllerFeedback::ConstPtr& msg);
static bool start_cb(lite_quad_uav::StartTask::Request& req, lite_quad_uav::StartTask::Response& res);
static bool control_cb(lite_quad_uav::ControlTask::Request& req, lite_quad_uav::ControlTask::Response& res);
static void runtime_check_timer_cb(const ros::TimerEvent& event);
static void update_timer_cb(const ros::TimerEvent& event);
static void log_report_timer_cb(const ros::TimerEvent& event);

/* Main function / 主函数 -----------------------------------------------------*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "square_patrol_task_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    load_params(pnh);

    // ── 话题与服务名称 ──────────────────────────────────────────────────
    const std::string vehicle_status_topic = "/uav/vehicle/status";           // 车辆连接状态 (connect_manager 发布)
    const std::string controller_feedback_topic = "/uav/control/controller_feedback"; // 控制器反馈 (controller 发布)
    const std::string control_request_topic = "/uav/control/request";         // 控制请求输出 (controller 消费)
    const std::string task_status_topic = "/uav/tasks/square_patrol/status";  // 任务状态 (task_manager 消费)
    const std::string log_report_topic = "/uav/log_reports/tasks/square_patrol"; // 节点日志报告 (logger_node 消费)
    const std::string start_srv_name = "/uav/tasks/square_patrol/start";      // 启动任务服务
    const std::string control_srv_name = "/uav/tasks/square_patrol/control";  // 控制任务服务 (PAUSE/RESUME/CANCEL)
    const std::string vehicle_command_srv_name = "/uav/vehicle/command";      // 车辆命令服务 (SET_MODE/ARM)

    local_pos_sub = nh.subscribe(mavros_ns + "/local_position/pose", 10, local_pos_cb);
    vehicle_status_sub = nh.subscribe(vehicle_status_topic, 10, vehicle_status_cb);
    controller_feedback_sub = nh.subscribe(controller_feedback_topic, 10, controller_feedback_cb);

    control_request_pub = nh.advertise<lite_quad_uav::ControlRequest>(control_request_topic, 10);
    status_pub = nh.advertise<lite_quad_uav::TaskStatus>(task_status_topic, 10);
    log_report_pub = nh.advertise<lite_quad_uav::TaskNodeLogReport>(log_report_topic, 10);

    start_srv = nh.advertiseService(start_srv_name, start_cb);
    control_srv = nh.advertiseService(control_srv_name, control_cb);
    vehicle_command_client = nh.serviceClient<lite_quad_uav::VehicleCommand>(vehicle_command_srv_name);

    runtime_check_timer = nh.createTimer(ros::Duration(1.0 / runtime_check_rate), runtime_check_timer_cb);
    update_timer = nh.createTimer(ros::Duration(1.0 / update_rate), update_timer_cb);
    log_report_timer = nh.createTimer(ros::Duration(1.0 / log_report_rate), log_report_timer_cb);

    if (auto_start) {
        activate_task(ros::Time::now(), "auto_start");
    }

    ROS_INFO("[Square_Patrol_Task_node] Initialized, side = %.2f, altitude = %.2f, update_rate = %.1f, runtime_check_rate = %.1f, auto_start = %d",
             square_side_length, flight_altitude, update_rate, runtime_check_rate, auto_start);

    ros::spin();
    return 0;
}

/* Private function implementations / 私有函数实现 ----------------------------*/

static void load_params(ros::NodeHandle& pnh) {
    mavros_ns = pnh.param<std::string>("mavros_ns", "/mavros");
    square_side_length = pnh.param<double>("square_side_length", 10.0);
    flight_altitude = pnh.param<double>("flight_altitude", 3.0);
    takeoff_height = pnh.param<double>("takeoff_height", 3.0);
    waypoint_accept_radius = pnh.param<double>("waypoint_accept_radius", 0.4);
    waypoint_hold_time = pnh.param<double>("waypoint_hold_time", 0.0);
    max_xy_speed = pnh.param<double>("max_xy_speed", 1.0);
    max_z_up_speed = pnh.param<double>("max_z_up_speed", 0.6);
    max_z_down_speed = pnh.param<double>("max_z_down_speed", 0.6);
    max_yaw_rate = pnh.param<double>("max_yaw_rate", 0.0);
    update_rate = pnh.param<double>("update_rate", 20.0);
    runtime_check_rate = pnh.param<double>("runtime_check_rate", 20.0);
    local_position_timeout = pnh.param<double>("local_position_timeout", 2.0);
    vehicle_status_timeout = pnh.param<double>("vehicle_status_timeout", 2.0);
    mode_confirm_timeout = pnh.param<double>("mode_confirm_timeout", 10.0);
    arm_confirm_timeout = pnh.param<double>("arm_confirm_timeout", 10.0);
    log_report_rate = pnh.param<double>("log_report_rate", 1.0);
    auto_start = pnh.param<bool>("auto_start", true);

    if (update_rate <= 0.0) {
        ROS_WARN("[Square_Patrol_Task_node] update_rate must be positive, reset to 20.0");
        update_rate = 20.0;
    }
    if (runtime_check_rate <= 0.0) {
        ROS_WARN("[Square_Patrol_Task_node] runtime_check_rate must be positive, reset to 20.0");
        runtime_check_rate = 20.0;
    }
    if (local_position_timeout <= 0.0) {
        ROS_WARN("[Square_Patrol_Task_node] local_position_timeout must be positive, reset to 2.0");
        local_position_timeout = 2.0;
    }
    if (vehicle_status_timeout <= 0.0) {
        ROS_WARN("[Square_Patrol_Task_node] vehicle_status_timeout must be positive, reset to 2.0");
        vehicle_status_timeout = 2.0;
    }
    if (mode_confirm_timeout <= 0.0) {
        ROS_WARN("[Square_Patrol_Task_node] mode_confirm_timeout must be positive, reset to 10.0");
        mode_confirm_timeout = 10.0;
    }
    if (arm_confirm_timeout <= 0.0) {
        ROS_WARN("[Square_Patrol_Task_node] arm_confirm_timeout must be positive, reset to 10.0");
        arm_confirm_timeout = 10.0;
    }
    if (log_report_rate <= 0.0) {
        ROS_WARN("[Square_Patrol_Task_node] log_report_rate must be positive, reset to 1.0");
        log_report_rate = 1.0;
    }
}

static void generate_waypoints(double origin_x, double origin_y) {
    waypoints.clear();
    waypoints.push_back(Waypoint(origin_x, origin_y, flight_altitude));
    waypoints.push_back(Waypoint(origin_x + square_side_length, origin_y, flight_altitude));
    waypoints.push_back(Waypoint(origin_x + square_side_length, origin_y + square_side_length, flight_altitude));
    waypoints.push_back(Waypoint(origin_x, origin_y + square_side_length, flight_altitude));
}

static const char* node_state_to_string(SquarePatrolNodeState state) {
    switch (state) {
    case SquarePatrolNodeState::INITIALIZING:   return "INITIALIZING";
    case SquarePatrolNodeState::NORMAL:         return "NORMAL";
    case SquarePatrolNodeState::REINITIALIZING: return "REINITIALIZING";
    }
    return "UNKNOWN";
}

static const char* task_progress_to_string(SquarePatrolTaskState progress) {
    switch (progress) {
    case TASK_IDLE:             return "IDLE";
    case TASK_PRECHECK:         return "PRECHECK";
    case TASK_MODE_REQUEST:     return "MODE_REQUEST";
    case TASK_MODE_WAIT_CONFIRM:return "MODE_WAIT_CONFIRM";
    case TASK_ARM_REQUEST:      return "ARM_REQUEST";
    case TASK_ARM_WAIT_CONFIRM: return "ARM_WAIT_CONFIRM";
    case TASK_TAKEOFF:          return "TAKEOFF";
    case TASK_PATROL:           return "PATROL";
    case TASK_HOLD:             return "HOLD";
    case TASK_PAUSED:           return "PAUSED";
    case TASK_CANCELED:         return "CANCELED";
    case TASK_FAILED:           return "FAILED";
    }
    return "UNKNOWN";
}

static void transition_node_to(SquarePatrolNodeState next_state) {
    if (node_state == next_state) {
        return;
    }

    ROS_INFO("[Square_Patrol_Task_node] node_state: %s -> %s",
             node_state_to_string(node_state), node_state_to_string(next_state));
    node_state = next_state;
}

static bool task_is_busy() {
    return task_state == "RUNNING" || task_state == "PAUSED";
}

static bool task_is_paused() {
    return task_state == "PAUSED";
}

static ActionInput make_action_input(const ros::Time& now) {
    ActionInput input;
    input.now = now;
    input.vehicle_state = vehicle_state;
    input.controller_feedback = controller_feedback;
    return input;
}

static void transition_to(SquarePatrolTaskState next_state) {
    if (task_progress == next_state) return;
    ROS_INFO("[Square_Patrol_Task_node] %s -> %s", task_progress_to_string(task_progress), task_progress_to_string(next_state));
    task_progress = next_state;
    state_enter_time = ros::Time::now();
}

static void activate_task(const ros::Time& now, const std::string& reason) {
    task_state = "RUNNING";
    task_message = "Task activated: " + reason;
    active_action.reset();
    pause_hold_action.reset();
    active_action_name = "";
    patrol_origin_locked = false;
    waypoints.clear();
    transition_to(TASK_PRECHECK);
    ROS_INFO("[Square_Patrol_Task_node] Activated");
    (void)now;
}

static void lock_patrol_origin() {
    if (patrol_origin_locked) return;
    patrol_origin_x = vehicle_state.x;
    patrol_origin_y = vehicle_state.y;
    patrol_origin_locked = true;
    generate_waypoints(patrol_origin_x, patrol_origin_y);
    ROS_INFO("[Square_Patrol_Task_node] Patrol origin locked, x = %.2f, y = %.2f", patrol_origin_x, patrol_origin_y);
}

static void enter_takeoff(const ros::Time& now) {
    lock_patrol_origin();

    TakeoffActionConfig config;
    config.source_task = "square_patrol";
    config.takeoff_height = takeoff_height;
    config.height_accept_error = waypoint_accept_radius;
    config.stable_time = 1.0;
    config.max_xy_speed = max_xy_speed;
    config.max_z_up_speed = max_z_up_speed;
    config.max_z_down_speed = max_z_down_speed;
    config.max_yaw_rate = max_yaw_rate;
    active_action.reset(new TakeoffAction(config));
    active_action->onEnter(now);
    active_action_name = "takeoff";
    transition_to(TASK_TAKEOFF);
}

static void enter_patrol(const ros::Time& now) {
    WaypointActionConfig config;
    config.source_task = "square_patrol";
    config.waypoints = waypoints;
    config.accept_radius = waypoint_accept_radius;
    config.hold_time = waypoint_hold_time;
    config.loop = true;
    config.max_xy_speed = max_xy_speed;
    config.max_z_up_speed = max_z_up_speed;
    config.max_z_down_speed = max_z_down_speed;
    config.max_yaw_rate = max_yaw_rate;
    active_action.reset(new WaypointAction(config));
    active_action->onEnter(now);
    active_action_name = "waypoint";
    transition_to(TASK_PATROL);
}

static void enter_paused(const ros::Time& now) {
    HoldActionConfig config;
    config.source_task = "square_patrol";
    config.max_xy_speed = max_xy_speed;
    config.max_z_up_speed = max_z_up_speed;
    config.max_z_down_speed = max_z_down_speed;
    config.max_yaw_rate = max_yaw_rate;
    pause_hold_action.reset(new HoldAction(config));
    pause_hold_action->onEnter(now);
    paused_from_state = task_progress;
    task_state = "PAUSED";
    active_action_name = "hold";
    transition_to(TASK_PAUSED);
}

static lite_quad_uav::ControlRequest make_idle_request(const ros::Time& now) {
    lite_quad_uav::ControlRequest request;
    request.header.stamp = now;
    request.source_task = "square_patrol";
    request.source_action = "idle";
    request.mode = "IDLE";
    request.valid = false;
    return request;
}

static void publish_output(const ActionOutput& output) {
    control_request_pub.publish(output.control_request);
}

static bool local_position_fresh(const ros::Time& now) {
    (void)now;
    return runtime_flags.local_position_fresh;
}

static bool vehicle_status_fresh(const ros::Time& now) {
    (void)now;
    return runtime_flags.vehicle_status_fresh;
}

static void update_runtime_check_flags(const ros::Time& now) {
    runtime_flags.local_position_fresh =
        local_position_received && (now - last_local_position_time).toSec() <= local_position_timeout;
    runtime_flags.vehicle_status_fresh =
        vehicle_status_received && (now - last_vehicle_status_time).toSec() <= vehicle_status_timeout;
    runtime_flags.vehicle_connected = runtime_flags.vehicle_status_fresh && vehicle_status.connected;
    runtime_flags.reliable_vehicle_connection = runtime_flags.vehicle_connected;
    runtime_flags.vehicle_ready = runtime_flags.vehicle_status_fresh && vehicle_status.ready;
    runtime_flags.vehicle_armed = runtime_flags.vehicle_status_fresh && vehicle_status.armed;

    vehicle_state.valid = runtime_flags.local_position_fresh;
    controller_feedback.valid = controller_feedback_received;
}

static void update_node_state(const ros::Time& /*now*/) {
    switch (node_state) {
    case SquarePatrolNodeState::INITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_node_to(SquarePatrolNodeState::NORMAL);
        }
        break;

    case SquarePatrolNodeState::NORMAL:
        if (!runtime_flags.reliable_vehicle_connection) {
            transition_node_to(SquarePatrolNodeState::REINITIALIZING);
        }
        break;

    case SquarePatrolNodeState::REINITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_node_to(SquarePatrolNodeState::NORMAL);
        }
        break;
    }
}

static void refresh_task_context(const ros::Time& now) {
    update_runtime_check_flags(now);
    update_node_state(now);
}

static bool precheck_ready(const ros::Time& now, std::string& reason) {
    if (node_state != SquarePatrolNodeState::NORMAL) {
        reason = "Task node is not ready: " + std::string(node_state_to_string(node_state));
        return false;
    }
    if (!runtime_flags.reliable_vehicle_connection) {
        reason = "Waiting for reliable vehicle connection";
        return false;
    }
    if (!local_position_fresh(now)) {
        reason = "Waiting for fresh local_position/pose";
        return false;
    }
    if (!vehicle_status_fresh(now)) {
        reason = "Waiting for vehicle status";
        return false;
    }
    if (!runtime_flags.vehicle_ready) {
        reason = "Waiting for vehicle ready: " + vehicle_status.message;
        return false;
    }
    return true;
}

static bool flight_runtime_ready(const ros::Time& now, std::string& reason) {
    if (!precheck_ready(now, reason)) {
        return false;
    }
    if (!vehicle_mode_is("OFFBOARD")) {
        reason = "Vehicle mode changed during task: " + vehicle_status.mode;
        return false;
    }
    if (!runtime_flags.vehicle_armed) {
        reason = "Vehicle disarmed during task";
        return false;
    }
    return true;
}

static bool vehicle_mode_is(const std::string& mode) {
    return runtime_flags.vehicle_status_fresh && vehicle_status.mode == mode;
}

static bool request_vehicle_mode(const std::string& mode, std::string& message) {
    lite_quad_uav::VehicleCommand srv;
    srv.request.command = "SET_MODE";
    srv.request.param = mode;
    if (!vehicle_command_client.call(srv)) {
        message = "Failed to call /uav/vehicle/command SET_MODE";
        return false;
    }
    message = srv.response.message;
    return srv.response.success;
}

static bool request_arm(std::string& message) {
    lite_quad_uav::VehicleCommand srv;
    srv.request.command = "ARM";
    srv.request.param = "";
    if (!vehicle_command_client.call(srv)) {
        message = "Failed to call /uav/vehicle/command ARM";
        return false;
    }
    message = srv.response.message;
    return srv.response.success;
}

static void fail_task(const std::string& reason) {
    if (active_action) active_action->onCancel(ros::Time::now());
    if (pause_hold_action) pause_hold_action->onCancel(ros::Time::now());
    active_action.reset();
    pause_hold_action.reset();
    task_state = "FAILED";
    active_action_name = "";
    task_message = reason;
    transition_to(TASK_FAILED);
}

static void cancel_task(const std::string& reason) {
    if (active_action) active_action->onCancel(ros::Time::now());
    if (pause_hold_action) pause_hold_action->onCancel(ros::Time::now());
    active_action.reset();
    pause_hold_action.reset();
    task_state = "CANCELED";
    active_action_name = "";
    task_message = reason;
    transition_to(TASK_CANCELED);
}

static void publish_status(const ros::Time& now) {
    lite_quad_uav::TaskStatus status;
    status.header.stamp = now;
    status.task_name = "square_patrol";
    status.task_state = task_state;
    status.task_progress = task_progress_to_string(task_progress);
    status.active_action = active_action_name;
    status.message = task_message;
    status_pub.publish(status);
}

static std::string normalize_command(const std::string& command) {
    std::string out = command;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return std::toupper(c); });
    return out;
}

static bool handle_pause(lite_quad_uav::ControlTask::Response& res) {
    if (!task_is_busy() || task_is_paused()) {
        res.accepted = false;
        res.message = "square_patrol is not running";
        return true;
    }
    if (task_progress != TASK_TAKEOFF && task_progress != TASK_PATROL) {
        res.accepted = false;
        res.message = "square_patrol cannot pause before flight action";
        return true;
    }
    enter_paused(ros::Time::now());
    task_message = "Task paused";
    res.accepted = true;
    res.message = "square_patrol paused";
    return true;
}

static bool handle_resume(lite_quad_uav::ControlTask::Response& res) {
    if (!task_is_paused()) {
        res.accepted = false;
        res.message = "square_patrol is not paused";
        return true;
    }
    if (pause_hold_action) {
        pause_hold_action->onExit(ros::Time::now());
        pause_hold_action.reset();
    }
    task_state = "RUNNING";
    active_action_name = paused_from_state == TASK_TAKEOFF ? "takeoff" : "waypoint";
    transition_to(paused_from_state);
    task_message = "Task resumed";
    res.accepted = true;
    res.message = "square_patrol resumed";
    return true;
}

static bool handle_cancel(lite_quad_uav::ControlTask::Response& res) {
    if (!task_is_busy()) {
        res.accepted = false;
        res.message = "square_patrol is not active";
        return true;
    }
    cancel_task("Task canceled");
    res.accepted = true;
    res.message = "square_patrol canceled";
    return true;
}

static void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    vehicle_state.x = msg->pose.position.x;
    vehicle_state.y = msg->pose.position.y;
    vehicle_state.z = msg->pose.position.z;
    last_local_position_time = ros::Time::now();
    local_position_received = true;
}

static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg) {
    vehicle_status = *msg;
    last_vehicle_status_time = ros::Time::now();
    vehicle_status_received = true;
}

static void controller_feedback_cb(const lite_quad_uav::ControllerFeedback::ConstPtr& msg) {
    controller_feedback.valid = true;
    controller_feedback.converged = msg->converged;
    controller_feedback.position_error_norm = msg->position_error_norm;
    last_controller_feedback_time = ros::Time::now();
    controller_feedback_received = true;
}

static bool start_cb(lite_quad_uav::StartTask::Request& req,
                     lite_quad_uav::StartTask::Response& res) {
    ros::Time now = ros::Time::now();
    refresh_task_context(now);

    if (task_is_busy()) {
        res.accepted = false;
        res.message = "square_patrol is already active";
        return true;
    }
    if (!req.task_type.empty() && req.task_type != "square_patrol") {
        res.accepted = false;
        res.message = "Unsupported task type: " + req.task_type;
        return true;
    }

    std::string reason;
    if (!precheck_ready(now, reason)) {
        res.accepted = false;
        res.message = reason;
        return true;
    }

    activate_task(now, "service");
    res.accepted = true;
    res.message = "square_patrol activated";
    return true;
}

static bool control_cb(lite_quad_uav::ControlTask::Request& req,
                       lite_quad_uav::ControlTask::Response& res) {
    refresh_task_context(ros::Time::now());
    std::string cmd = normalize_command(req.command);

    if (cmd != "CANCEL" && node_state != SquarePatrolNodeState::NORMAL) {
        res.accepted = false;
        res.message = "square_patrol task node is not ready: " + std::string(node_state_to_string(node_state));
        return true;
    }

    if (cmd == "PAUSE") {
        return handle_pause(res);
    }
    if (cmd == "RESUME") {
        return handle_resume(res);
    }
    if (cmd == "CANCEL") {
        return handle_cancel(res);
    }

    res.accepted = false;
    res.message = "Unsupported control command: " + req.command;
    return true;
}

static void runtime_check_timer_cb(const ros::TimerEvent& /*event*/) {
    update_runtime_check_flags(ros::Time::now());
}

static void update_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    refresh_task_context(now);

    ActionInput input = make_action_input(now);
    ActionOutput output;

    if (node_state == SquarePatrolNodeState::REINITIALIZING && task_is_busy()) {
        fail_task("Vehicle reliable connection lost, task interrupted");
        output.status = ActionStatus::CANCELED;
        output.control_request = make_idle_request(now);
        publish_output(output);
        publish_status(now);
        return;
    }

    switch (task_progress) {
    case TASK_IDLE:
        output.status = ActionStatus::IDLE;
        output.control_request = make_idle_request(now);
        publish_output(output);
        break;

    case TASK_PRECHECK:
    {
        std::string reason;
        if (precheck_ready(now, reason)) {
            task_message = "Precheck passed, requesting OFFBOARD mode";
            active_action_name = "set_mode";
            transition_to(TASK_MODE_REQUEST);
        } else {
            task_message = reason;
            output.status = ActionStatus::RUNNING;
            output.control_request = make_idle_request(now);
            publish_output(output);
        }
        break;
    }

    case TASK_MODE_REQUEST:
    {
        std::string reason;
        if (!precheck_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
            break;
        }

        active_action_name = "set_mode";
        if (vehicle_mode_is("OFFBOARD")) {
            task_message = "OFFBOARD mode already active";
            transition_to(TASK_ARM_REQUEST);
        } else {
            std::string mode_message;
            if (request_vehicle_mode("OFFBOARD", mode_message)) {
                task_message = "OFFBOARD request accepted";
                transition_to(TASK_MODE_WAIT_CONFIRM);
            } else {
                fail_task("OFFBOARD request failed: " + mode_message);
                output.status = ActionStatus::CANCELED;
                output.control_request = make_idle_request(now);
                publish_output(output);
            }
        }
        break;
    }

    case TASK_MODE_WAIT_CONFIRM:
    {
        std::string reason;
        if (!precheck_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
        } else if (vehicle_mode_is("OFFBOARD")) {
            task_message = "OFFBOARD mode confirmed";
            transition_to(TASK_ARM_REQUEST);
        } else if ((now - state_enter_time).toSec() > mode_confirm_timeout) {
            fail_task("OFFBOARD confirmation timeout");
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
        } else {
            task_message = "Waiting for OFFBOARD confirmation";
            output.status = ActionStatus::RUNNING;
            output.control_request = make_idle_request(now);
            publish_output(output);
        }
        break;
    }

    case TASK_ARM_REQUEST:
    {
        std::string reason;
        if (!precheck_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
            break;
        }

        if (!vehicle_mode_is("OFFBOARD")) {
            task_message = "OFFBOARD lost before ARM, requesting mode again";
            active_action_name = "set_mode";
            transition_to(TASK_MODE_REQUEST);
            break;
        }

        active_action_name = "arm";
        if (runtime_flags.vehicle_armed) {
            task_message = "Vehicle already armed";
            enter_takeoff(now);
        } else {
            std::string arm_message;
            if (request_arm(arm_message)) {
                task_message = "ARM request accepted";
                transition_to(TASK_ARM_WAIT_CONFIRM);
            } else {
                fail_task("ARM request failed: " + arm_message);
                output.status = ActionStatus::CANCELED;
                output.control_request = make_idle_request(now);
                publish_output(output);
            }
        }
        break;
    }

    case TASK_ARM_WAIT_CONFIRM:
    {
        std::string reason;
        if (!precheck_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
        } else if (runtime_flags.vehicle_armed) {
            task_message = "Vehicle armed";
            enter_takeoff(now);
        } else if ((now - state_enter_time).toSec() > arm_confirm_timeout) {
            fail_task("ARM confirmation timeout");
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
        } else {
            task_message = "Waiting for ARM confirmation";
            output.status = ActionStatus::RUNNING;
            output.control_request = make_idle_request(now);
            publish_output(output);
        }
        break;
    }

    case TASK_TAKEOFF:
    {
        std::string reason;
        if (!flight_runtime_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
            break;
        }
        if (!active_action) enter_takeoff(now);
        output = active_action->onUpdate(input);
        publish_output(output);
        task_message = "Taking off";
        if (output.status == ActionStatus::SUCCESS) {
            active_action->onExit(now);
            enter_patrol(now);
            task_message = "Takeoff finished, patrol started";
        } else if (output.status == ActionStatus::FAILED ||
                   output.status == ActionStatus::CANCELED ||
                   output.status == ActionStatus::TIMEOUT) {
            fail_task("Takeoff action failed");
        }
        break;
    }

    case TASK_PATROL:
    {
        std::string reason;
        if (!flight_runtime_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
            break;
        }
        if (!active_action) enter_patrol(now);
        output = active_action->onUpdate(input);
        publish_output(output);
        task_message = "Patrolling square waypoints";
        if (output.status == ActionStatus::FAILED ||
            output.status == ActionStatus::CANCELED ||
            output.status == ActionStatus::TIMEOUT) {
            fail_task("Waypoint action failed");
        }
        break;
    }

    case TASK_HOLD:
    {
        std::string reason;
        if (!flight_runtime_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
            break;
        }
        if (!pause_hold_action) enter_paused(now);
        output = pause_hold_action->onUpdate(input);
        publish_output(output);
        task_message = "Holding position";
        break;
    }

    case TASK_PAUSED:
    {
        std::string reason;
        if (!flight_runtime_ready(now, reason)) {
            fail_task(reason);
            output.status = ActionStatus::CANCELED;
            output.control_request = make_idle_request(now);
            publish_output(output);
            break;
        }
        if (!pause_hold_action) enter_paused(now);
        output = pause_hold_action->onUpdate(input);
        publish_output(output);
        task_message = "Task paused, holding position";
        break;
    }

    case TASK_CANCELED:
    case TASK_FAILED:
        output.status = ActionStatus::CANCELED;
        output.control_request = make_idle_request(now);
        publish_output(output);
        break;
    }

    publish_status(now);
}

static void log_report_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    refresh_task_context(now);

    lite_quad_uav::TaskNodeLogReport report;
    report.header.stamp = now;
    report.task_name = "square_patrol";
    report.node_state = node_state_to_string(node_state);
    report.task_state = task_state;
    report.task_progress = task_progress_to_string(task_progress);
    report.active_action = active_action_name;
    report.local_position_fresh = runtime_flags.local_position_fresh;
    report.vehicle_status_fresh = runtime_flags.vehicle_status_fresh;
    report.reliable_vehicle_connection = runtime_flags.reliable_vehicle_connection;
    report.vehicle_ready = runtime_flags.vehicle_ready;
    report.vehicle_armed = runtime_flags.vehicle_armed;
    report.message = task_message;

    log_report_pub.publish(report);
}
