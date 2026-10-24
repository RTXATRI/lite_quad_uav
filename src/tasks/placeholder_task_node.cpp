/**
 * @file placeholder_task_node.cpp
 * @brief 空任务占位节点：提供统一 TaskNode 接口，不参与飞控控制链路。
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <algorithm>
#include <cctype>
#include <string>

#include <ros/ros.h>

#include <lite_quad_uav/ControlTask.h>
#include <lite_quad_uav/StartTask.h>
#include <lite_quad_uav/TaskNodeLogReport.h>
#include <lite_quad_uav/TaskStatus.h>
#include <lite_quad_uav/VehicleStatus.h>

/* Private defines / 私有宏定义 ----------------------------------------------*/

#ifndef DEFAULT_TASK_ID
#define DEFAULT_TASK_ID "placeholder"
#endif

#ifndef DEFAULT_TASK_NS
#define DEFAULT_TASK_NS "/uav/tasks/placeholder"
#endif

/* Private typedef / 私有类型定义 --------------------------------------------*/

enum PlaceholderTaskState {
    PLACEHOLDER_IDLE = 0,
    PLACEHOLDER_RUNNING,
    PLACEHOLDER_SUCCEEDED,
    PLACEHOLDER_CANCELED
};

enum class PlaceholderNodeState {
    INITIALIZING = 0,
    NORMAL,
    REINITIALIZING
};

struct PlaceholderRuntimeFlags {
    bool vehicle_status_fresh = false;
    bool vehicle_connected = false;
    bool reliable_vehicle_connection = false;
    bool vehicle_ready = false;
    bool vehicle_armed = false;
};

/* Private variables / 私有变量 ----------------------------------------------*/

static PlaceholderTaskState task_progress = PLACEHOLDER_IDLE;
static PlaceholderNodeState node_state = PlaceholderNodeState::INITIALIZING;

static std::string task_id = DEFAULT_TASK_ID;
static std::string task_ns = DEFAULT_TASK_NS;
static double runtime_check_rate = 20.0;
static double vehicle_status_timeout = 2.0;
static double status_publish_rate = 2.0;
static double log_report_rate = 1.0;
static double finish_delay = 0.0;
static bool auto_start = false;

static std::string task_state = "IDLE";
static std::string task_message = "Placeholder task idle";
static ros::Time state_enter_time;
static lite_quad_uav::VehicleStatus vehicle_status;
static bool vehicle_status_received = false;
static ros::Time last_vehicle_status_time;
static PlaceholderRuntimeFlags runtime_flags;

static ros::Subscriber vehicle_status_sub;
static ros::Publisher status_pub;
static ros::Publisher log_report_pub;
static ros::ServiceServer start_srv;
static ros::ServiceServer control_srv;
static ros::Timer runtime_check_timer;
static ros::Timer status_timer;
static ros::Timer log_report_timer;

/* Private function prototypes / 私有函数声明 --------------------------------*/

static void load_params(ros::NodeHandle& pnh);
static const char* node_state_to_string(PlaceholderNodeState state);
static const char* task_progress_to_string(PlaceholderTaskState progress);
static bool task_is_busy();
static const char* task_progress_string();
static std::string normalize_command(const std::string& command);
static void transition_node_to(PlaceholderNodeState next_state);
static void transition_to(PlaceholderTaskState next_state);
static void update_runtime_check_flags(const ros::Time& now);
static void update_node_state(const ros::Time& now);
static void refresh_placeholder_context(const ros::Time& now);
static void activate_task(const ros::Time& now, const std::string& reason);
static void finish_task(const ros::Time& now);
static void cancel_task(const std::string& reason);
static void publish_status(const ros::Time& now);
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg);
static bool start_cb(lite_quad_uav::StartTask::Request& req, lite_quad_uav::StartTask::Response& res);
static bool control_cb(lite_quad_uav::ControlTask::Request& req, lite_quad_uav::ControlTask::Response& res);
static void runtime_check_timer_cb(const ros::TimerEvent& event);
static void status_timer_cb(const ros::TimerEvent& event);
static void log_report_timer_cb(const ros::TimerEvent& event);

/* Main function / 主函数 -----------------------------------------------------*/

int main(int argc, char** argv) {
    std::string default_node_name = task_id + "_task_node";
    ros::init(argc, argv, default_node_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    load_params(pnh);

    // ── 话题与服务名称 ──────────────────────────────────────────────────
    const std::string vehicle_status_topic = "/uav/vehicle/status";            // 车辆连接状态 (connect_manager 发布)
    const std::string log_report_topic = "/uav/log_reports/tasks/" + task_id;  // 本任务日志报告 (logger_node 消费)

    vehicle_status_sub = nh.subscribe(vehicle_status_topic, 10, vehicle_status_cb);
    status_pub = nh.advertise<lite_quad_uav::TaskStatus>(task_ns + "/status", 10);
    log_report_pub = nh.advertise<lite_quad_uav::TaskNodeLogReport>(log_report_topic, 10);
    start_srv = nh.advertiseService(task_ns + "/start", start_cb);
    control_srv = nh.advertiseService(task_ns + "/control", control_cb);
    runtime_check_timer = nh.createTimer(ros::Duration(1.0 / runtime_check_rate), runtime_check_timer_cb);
    status_timer = nh.createTimer(ros::Duration(1.0 / status_publish_rate), status_timer_cb);
    log_report_timer = nh.createTimer(ros::Duration(1.0 / log_report_rate), log_report_timer_cb);

    if (auto_start) {
        activate_task(ros::Time::now(), "auto_start");
    }

    ROS_INFO("[Placeholder_Task_node] Initialized task_id=%s task_ns=%s runtime_check_rate=%.1f auto_start=%d",
             task_id.c_str(), task_ns.c_str(), runtime_check_rate, auto_start);

    ros::spin();
    return 0;
}

/* Private function implementations / 私有函数实现 ----------------------------*/

static void load_params(ros::NodeHandle& pnh) {
    task_id = pnh.param<std::string>("task_id", std::string(DEFAULT_TASK_ID));
    task_ns = pnh.param<std::string>("task_ns", std::string(DEFAULT_TASK_NS));
    runtime_check_rate = pnh.param<double>("runtime_check_rate", 20.0);
    vehicle_status_timeout = pnh.param<double>("vehicle_status_timeout", 2.0);
    status_publish_rate = pnh.param<double>("status_publish_rate", 2.0);
    log_report_rate = pnh.param<double>("log_report_rate", 1.0);
    finish_delay = pnh.param<double>("finish_delay", 0.0);
    auto_start = pnh.param<bool>("auto_start", false);

    if (runtime_check_rate <= 0.0) {
        ROS_WARN("[Placeholder_Task_node] runtime_check_rate must be positive, reset to 20.0");
        runtime_check_rate = 20.0;
    }
    if (vehicle_status_timeout <= 0.0) {
        ROS_WARN("[Placeholder_Task_node] vehicle_status_timeout must be positive, reset to 2.0");
        vehicle_status_timeout = 2.0;
    }
    if (status_publish_rate <= 0.0) {
        ROS_WARN("[Placeholder_Task_node] status_publish_rate must be positive, reset to 2.0");
        status_publish_rate = 2.0;
    }
    if (log_report_rate <= 0.0) {
        ROS_WARN("[Placeholder_Task_node] log_report_rate must be positive, reset to 1.0");
        log_report_rate = 1.0;
    }
    if (finish_delay < 0.0) {
        ROS_WARN("[Placeholder_Task_node] finish_delay must be non-negative, reset to 0.0");
        finish_delay = 0.0;
    }
}

static const char* node_state_to_string(PlaceholderNodeState state) {
    switch (state) {
    case PlaceholderNodeState::INITIALIZING:   return "INITIALIZING";
    case PlaceholderNodeState::NORMAL:         return "NORMAL";
    case PlaceholderNodeState::REINITIALIZING: return "REINITIALIZING";
    }
    return "UNKNOWN";
}

static const char* task_progress_to_string(PlaceholderTaskState progress) {
    switch (progress) {
    case PLACEHOLDER_IDLE:      return "IDLE";
    case PLACEHOLDER_RUNNING:   return "RUNNING";
    case PLACEHOLDER_SUCCEEDED: return "SUCCEEDED";
    case PLACEHOLDER_CANCELED:  return "CANCELED";
    }
    return "UNKNOWN";
}

static bool task_is_busy() {
    return task_state == "RUNNING" || task_state == "PAUSED";
}

static const char* task_progress_string() {
    return task_progress_to_string(task_progress);
}

static std::string normalize_command(const std::string& command) {
    std::string out = command;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return std::toupper(c); });
    return out;
}

static void transition_node_to(PlaceholderNodeState next_state) {
    if (node_state == next_state) {
        return;
    }

    ROS_INFO("[Placeholder_Task_node] %s node_state: %s -> %s",
             task_id.c_str(), node_state_to_string(node_state), node_state_to_string(next_state));
    node_state = next_state;
}

static void transition_to(PlaceholderTaskState next_state) {
    if (task_progress == next_state) return;
    ROS_INFO("[Placeholder_Task_node] %s: %s -> %s",
             task_id.c_str(), task_progress_to_string(task_progress), task_progress_to_string(next_state));
    task_progress = next_state;
    state_enter_time = ros::Time::now();
}

static void update_runtime_check_flags(const ros::Time& now) {
    runtime_flags.vehicle_status_fresh =
        vehicle_status_received && (now - last_vehicle_status_time).toSec() <= vehicle_status_timeout;
    runtime_flags.vehicle_connected = runtime_flags.vehicle_status_fresh && vehicle_status.connected;
    runtime_flags.reliable_vehicle_connection = runtime_flags.vehicle_connected;
    runtime_flags.vehicle_ready = runtime_flags.vehicle_status_fresh && vehicle_status.ready;
    runtime_flags.vehicle_armed = runtime_flags.vehicle_status_fresh && vehicle_status.armed;
}

static void update_node_state(const ros::Time& /*now*/) {
    switch (node_state) {
    case PlaceholderNodeState::INITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_node_to(PlaceholderNodeState::NORMAL);
        }
        break;

    case PlaceholderNodeState::NORMAL:
        if (!runtime_flags.reliable_vehicle_connection) {
            transition_node_to(PlaceholderNodeState::REINITIALIZING);
        }
        break;

    case PlaceholderNodeState::REINITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_node_to(PlaceholderNodeState::NORMAL);
        }
        break;
    }
}

static void refresh_placeholder_context(const ros::Time& now) {
    update_runtime_check_flags(now);
    update_node_state(now);
}

static void activate_task(const ros::Time& now, const std::string& reason) {
    task_state = "RUNNING";
    task_message = "Placeholder task activated: " + reason;
    transition_to(PLACEHOLDER_RUNNING);
    state_enter_time = now;
}

static void finish_task(const ros::Time& now) {
    task_state = "SUCCEEDED";
    task_message = "Placeholder task finished";
    transition_to(PLACEHOLDER_SUCCEEDED);
    state_enter_time = now;
}

static void cancel_task(const std::string& reason) {
    task_state = "CANCELED";
    task_message = reason;
    transition_to(PLACEHOLDER_CANCELED);
}

static void publish_status(const ros::Time& now) {
    lite_quad_uav::TaskStatus status;
    status.header.stamp = now;
    status.task_name = task_id;
    status.task_state = task_state;
    status.task_progress = task_progress_string();
    status.active_action = "";
    status.message = task_message;
    status_pub.publish(status);
}

static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg) {
    vehicle_status = *msg;
    last_vehicle_status_time = ros::Time::now();
    vehicle_status_received = true;
}

static bool start_cb(lite_quad_uav::StartTask::Request& req,
                     lite_quad_uav::StartTask::Response& res) {
    refresh_placeholder_context(ros::Time::now());

    if (task_is_busy()) {
        res.accepted = false;
        res.message = task_id + " is already active";
        return true;
    }
    if (!req.task_type.empty() && req.task_type != task_id) {
        res.accepted = false;
        res.message = "Unsupported task type: " + req.task_type;
        return true;
    }
    if (node_state != PlaceholderNodeState::NORMAL) {
        res.accepted = false;
        res.message = task_id + " task node is not ready: " + node_state_to_string(node_state);
        return true;
    }

    activate_task(ros::Time::now(), "service");
    res.accepted = true;
    res.message = task_id + " placeholder accepted";
    return true;
}

static bool control_cb(lite_quad_uav::ControlTask::Request& req,
                       lite_quad_uav::ControlTask::Response& res) {
    refresh_placeholder_context(ros::Time::now());
    std::string cmd = normalize_command(req.command);

    if (cmd == "CANCEL") {
        if (!task_is_busy()) {
            res.accepted = false;
            res.message = task_id + " is not active";
            return true;
        }
        cancel_task("Placeholder task canceled");
        res.accepted = true;
        res.message = task_id + " placeholder canceled";
        return true;
    }

    if (cmd == "PAUSE" || cmd == "RESUME") {
        res.accepted = false;
        res.message = task_id + " placeholder does not implement " + cmd;
        return true;
    }

    res.accepted = false;
    res.message = "Unsupported control command: " + req.command;
    return true;
}

static void runtime_check_timer_cb(const ros::TimerEvent& /*event*/) {
    update_runtime_check_flags(ros::Time::now());
}

static void status_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    refresh_placeholder_context(now);
    if (node_state == PlaceholderNodeState::REINITIALIZING && task_progress == PLACEHOLDER_RUNNING) {
        cancel_task("Vehicle reliable connection lost, placeholder canceled");
    }
    if (task_progress == PLACEHOLDER_RUNNING && (now - state_enter_time).toSec() >= finish_delay) {
        finish_task(now);
    }
    publish_status(now);
}

static void log_report_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    refresh_placeholder_context(now);

    lite_quad_uav::TaskNodeLogReport report;
    report.header.stamp = now;
    report.task_name = task_id;
    report.node_state = node_state_to_string(node_state);
    report.task_state = task_state;
    report.task_progress = task_progress_string();
    report.active_action = "";
    report.local_position_fresh = false;
    report.vehicle_status_fresh = runtime_flags.vehicle_status_fresh;
    report.reliable_vehicle_connection = runtime_flags.reliable_vehicle_connection;
    report.vehicle_ready = runtime_flags.vehicle_ready;
    report.vehicle_armed = runtime_flags.vehicle_armed;
    report.message = task_message;

    log_report_pub.publish(report);
}
