/**
 * @file task_manager_node.cpp
 * @brief 通用任务管理节点：注册、启动、控制常驻 TaskNode，汇总状态。
 *
 * NOTE: TaskManager 不执行任务内部状态机，只做任务节点管理和状态汇总。
 *       新增任务只需在 main() 中调用 register_task() 即可。
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <boost/bind.hpp>

#include <lite_quad_uav/ControlTask.h>
#include <lite_quad_uav/StartTask.h>
#include <lite_quad_uav/TaskManagerLogReport.h>
#include <lite_quad_uav/TaskManagerStatus.h>
#include <lite_quad_uav/TaskStatus.h>
#include <lite_quad_uav/VehicleStatus.h>

/* Private typedef / 私有类型定义 --------------------------------------------*/

/**
 * TaskEntry 描述一个已注册的任务节点。
 * 每个任务节点必须遵守统一接口：
 *   - start service:  <task_ns>/start   (StartTask)
 *   - control service: <task_ns>/control (ControlTask)
 *   - status topic:   <task_ns>/status  (TaskStatus)
 */
struct TaskEntry {
    std::string task_id;          /* 任务标识，如 "square_patrol"          */
    std::string task_ns;          /* 任务命名空间，如 "/uav/tasks/square_patrol" */

    ros::ServiceClient start_client;    /* -> <task_ns>/start             */
    ros::ServiceClient control_client;  /* -> <task_ns>/control           */
    ros::Subscriber status_sub;         /* -> <task_ns>/status            */

    lite_quad_uav::TaskStatus last_status;
    bool status_received;
    ros::Time last_status_time;
};

enum class ManagedTaskState {
    IDLE = 0,
    RUNNING,
    PAUSED,
    CANCELING,
    SUCCEEDED,
    CANCELED,
    FAILED
};

enum class TaskManagerCommand {
    UNKNOWN = 0,
    PAUSE,
    RESUME,
    CANCEL
};

enum class TaskManagerNodeState {
    INITIALIZING = 0,
    NORMAL,
    REINITIALIZING
};

struct TaskManagerRuntimeFlags {
    bool vehicle_status_fresh = false;
    bool vehicle_connected = false;
    bool reliable_vehicle_connection = false;
    bool vehicle_ready = false;
    bool active_task_status_received = false;
    bool active_task_status_fresh = false;
};

/* Private variables / 私有变量 ----------------------------------------------*/

static std::map<std::string, TaskEntry> task_table;
static std::vector<std::string> registered_task_order;
static std::string active_task_id;     /* 空串 = 无活跃任务 */
static ManagedTaskState active_task_run_state = ManagedTaskState::IDLE;
static TaskManagerNodeState node_state = TaskManagerNodeState::INITIALIZING;

static double runtime_check_rate = 20.0;
static double status_publish_rate = 2.0;
static double vehicle_status_timeout = 2.0;
static double task_status_timeout = 3.0;
static double log_report_rate = 1.0;

static lite_quad_uav::VehicleStatus vehicle_status;
static bool vehicle_status_received = false;
static ros::Time last_vehicle_status_time;
static TaskManagerRuntimeFlags runtime_flags;

static ros::Publisher status_pub;
static ros::Publisher log_report_pub;
static ros::ServiceServer start_srv;
static ros::ServiceServer control_srv;
static ros::Subscriber vehicle_status_sub;

/* Private function prototypes / 私有函数声明 --------------------------------*/

static std::string normalize(const std::string& text);
static const char* task_state_to_string(ManagedTaskState state);
static const char* node_state_to_string(TaskManagerNodeState state);
static TaskManagerCommand parse_control_command(const std::string& command);
static ManagedTaskState task_state_from_string(const std::string& state);
static bool task_state_is_busy(ManagedTaskState state);
static bool task_busy();
static std::vector<std::string> registered_task_ids();
static bool accepting_new_task();
static std::string task_manager_message();
static void transition_to(TaskManagerNodeState next_state);
static void update_runtime_check_flags(const ros::Time& now);
static void sync_managed_task_state(const ros::Time& now);
static void update_node_state(const ros::Time& now);
static void refresh_task_manager_context(const ros::Time& now);
static bool task_start_prerequisites_met(std::string& reason);
static void register_task(ros::NodeHandle& nh, const std::string& task_id, const std::string& task_ns);
static void task_status_cb(const lite_quad_uav::TaskStatus::ConstPtr& msg, const std::string& task_id);
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg);
static lite_quad_uav::TaskManagerStatus make_task_manager_status(const ros::Time& now);
static void publish_task_manager_status(const ros::Time& now);
static void runtime_check_timer_cb(const ros::TimerEvent& event);
static void status_timer_cb(const ros::TimerEvent& event);
static void log_report_timer_cb(const ros::TimerEvent& event);
static bool start_task_cb(lite_quad_uav::StartTask::Request& req, lite_quad_uav::StartTask::Response& res);
static bool control_task_cb(lite_quad_uav::ControlTask::Request& req, lite_quad_uav::ControlTask::Response& res);

/* Main function / 主函数 -----------------------------------------------------*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("runtime_check_rate", runtime_check_rate, 20.0);
    pnh.param<double>("status_publish_rate", status_publish_rate, 2.0);
    pnh.param<double>("vehicle_status_timeout", vehicle_status_timeout, 2.0);
    pnh.param<double>("task_status_timeout", task_status_timeout, 3.0);
    pnh.param<double>("log_report_rate", log_report_rate, 1.0);
    if (runtime_check_rate <= 0.0) {
        ROS_WARN("[Task_Manager_node] runtime_check_rate must be positive, reset to 20.0");
        runtime_check_rate = 20.0;
    }
    if (status_publish_rate <= 0.0) {
        ROS_WARN("[Task_Manager_node] status_publish_rate must be positive, reset to 2.0");
        status_publish_rate = 2.0;
    }
    if (vehicle_status_timeout <= 0.0) {
        ROS_WARN("[Task_Manager_node] vehicle_status_timeout must be positive, reset to 2.0");
        vehicle_status_timeout = 2.0;
    }
    if (task_status_timeout <= 0.0) {
        ROS_WARN("[Task_Manager_node] task_status_timeout must be positive, reset to 3.0");
        task_status_timeout = 3.0;
    }
    if (log_report_rate <= 0.0) {
        ROS_WARN("[Task_Manager_node] log_report_rate must be positive, reset to 1.0");
        log_report_rate = 1.0;
    }

    // ── 话题与服务名称 ──────────────────────────────────────────────────
    const std::string vehicle_status_topic = "/uav/vehicle/status";           // 车辆连接状态 (connect_manager 发布)
    const std::string task_manager_status_topic = "/uav/task_manager/status";  // 任务管理器聚合状态 (周期发布)
    const std::string log_report_topic = "/uav/log_reports/task_manager";     // 节点日志报告 (logger_node 消费)
    const std::string start_task_srv_name = "/uav/task_manager/start_task";   // 启动任务服务 (GCS/外部调用)
    const std::string control_task_srv_name = "/uav/task_manager/control_task"; // 控制任务服务 (PAUSE/RESUME/CANCEL)

    // ── 注册任务命名空间 ──────────────────────────────────────────────
    // 每个任务节点必须遵守统一接口：start service / control service / status topic。
    const std::string square_patrol_ns = "/uav/tasks/square_patrol";          // 方形巡航任务命名空间
    const std::string hover_placeholder_ns = "/uav/tasks/hover_placeholder";  // 悬停占位任务命名空间
    const std::string inspection_placeholder_ns = "/uav/tasks/inspection_placeholder"; // 检查占位任务命名空间

    /* 注册所有已知任务。新增任务只需在此添加一行 register_task 调用。
     * NOTE: TaskManager 不执行任务内部状态机，只做任务节点管理和状态汇总。 */
    register_task(nh, "square_patrol", square_patrol_ns);
    register_task(nh, "hover_placeholder", hover_placeholder_ns);
    register_task(nh, "inspection_placeholder", inspection_placeholder_ns);

    /* 对外接口 */
    vehicle_status_sub = nh.subscribe(vehicle_status_topic, 10, vehicle_status_cb);
    status_pub = nh.advertise<lite_quad_uav::TaskManagerStatus>(task_manager_status_topic, 10);
    log_report_pub = nh.advertise<lite_quad_uav::TaskManagerLogReport>(log_report_topic, 10);
    start_srv  = nh.advertiseService(start_task_srv_name, start_task_cb);
    control_srv = nh.advertiseService(control_task_srv_name, control_task_cb);
    ros::Timer runtime_check_timer = nh.createTimer(ros::Duration(1.0 / runtime_check_rate), runtime_check_timer_cb);
    ros::Timer status_timer = nh.createTimer(ros::Duration(1.0 / status_publish_rate), status_timer_cb);
    ros::Timer log_report_timer = nh.createTimer(ros::Duration(1.0 / log_report_rate), log_report_timer_cb);

    ROS_INFO("[Task_Manager_node] Initialized, runtime_check_rate = %.1f, status_publish_rate = %.1f, registered_tasks = %zu",
             runtime_check_rate, status_publish_rate, task_table.size());

    ros::spin();

    return 0;
}

/* Private function implementations / 私有函数实现 ----------------------------*/

/**
 * @brief 字符串转大写，用于不区分大小写的命令匹配。
 * @param[in] text 原始字符串。
 * @return 大写化后的字符串。
 */
static std::string normalize(const std::string& text) {
    std::string out = text;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return std::toupper(c); });
    return out;
}

/**
 * @brief 将托管任务状态枚举转为字符串。
 * @param[in] state 状态枚举值。
 * @return 状态名称字符串。
 */
static const char* task_state_to_string(ManagedTaskState state) {
    switch (state) {
    case ManagedTaskState::IDLE:     return "IDLE";
    case ManagedTaskState::RUNNING:  return "RUNNING";
    case ManagedTaskState::PAUSED:   return "PAUSED";
    case ManagedTaskState::CANCELING: return "CANCELING";
    case ManagedTaskState::SUCCEEDED: return "SUCCEEDED";
    case ManagedTaskState::CANCELED:  return "CANCELED";
    case ManagedTaskState::FAILED:   return "FAILED";
    }
    return "UNKNOWN";
}

static const char* node_state_to_string(TaskManagerNodeState state) {
    switch (state) {
    case TaskManagerNodeState::INITIALIZING:   return "INITIALIZING";
    case TaskManagerNodeState::NORMAL:         return "NORMAL";
    case TaskManagerNodeState::REINITIALIZING: return "REINITIALIZING";
    }
    return "UNKNOWN";
}

/**
 * @brief 将命令字符串解析为 TaskManagerCommand 枚举。
 * @param[in] command 命令字符串（不区分大小写）。
 * @return 解析后的枚举值，无法识别时返回 UNKNOWN。
 */
static TaskManagerCommand parse_control_command(const std::string& command) {
    std::string normalized = normalize(command);
    if (normalized == "PAUSE")  return TaskManagerCommand::PAUSE;
    if (normalized == "RESUME") return TaskManagerCommand::RESUME;
    if (normalized == "CANCEL") return TaskManagerCommand::CANCEL;
    return TaskManagerCommand::UNKNOWN;
}

static ManagedTaskState task_state_from_string(const std::string& state) {
    std::string normalized = normalize(state);
    if (normalized == "RUNNING")   return ManagedTaskState::RUNNING;
    if (normalized == "PAUSED")    return ManagedTaskState::PAUSED;
    if (normalized == "SUCCEEDED") return ManagedTaskState::SUCCEEDED;
    if (normalized == "CANCELED")  return ManagedTaskState::CANCELED;
    if (normalized == "FAILED")    return ManagedTaskState::FAILED;
    return ManagedTaskState::IDLE;
}

static bool task_state_is_busy(ManagedTaskState state) {
    return state == ManagedTaskState::RUNNING ||
           state == ManagedTaskState::PAUSED ||
           state == ManagedTaskState::CANCELING;
}

/**
 * @brief 判断当前是否有任务占用管理器（不可启动新任务）。
 * @return true=有任务占用。
 *
 * @note PAUSED 状态也算 busy，会阻止启动新任务。
 */
static bool task_busy() {
    return !active_task_id.empty() && task_state_is_busy(active_task_run_state);
}

/**
 * @brief 返回已注册任务列表。
 * @return 任务 ID 列表。
 */
static std::vector<std::string> registered_task_ids() {
    return registered_task_order;
}

static bool accepting_new_task() {
    return node_state == TaskManagerNodeState::NORMAL &&
           runtime_flags.vehicle_ready &&
           !task_busy();
}

static std::string task_manager_message() {
    if (node_state == TaskManagerNodeState::INITIALIZING) {
        return "Waiting for reliable vehicle status";
    }
    if (node_state == TaskManagerNodeState::REINITIALIZING) {
        return "Vehicle status lost, waiting for reconnect";
    }
    if (!runtime_flags.vehicle_ready) {
        return "Vehicle connected but not ready for task";
    }
    if (task_busy()) {
        return "Task manager busy";
    }
    return "Task manager ready";
}

/**
 * @brief 切换 task_manager 自身状态。
 * @param[in] next_state 目标状态。
 */
static void transition_to(TaskManagerNodeState next_state) {
    if (node_state == next_state) {
        return;
    }

    ROS_INFO("[Task_Manager_node] node_state: %s -> %s",
             node_state_to_string(node_state), node_state_to_string(next_state));
    node_state = next_state;
}

/**
 * @brief 统一刷新运行时健康标志。
 * @param[in] now 当前时间。
 *
 * @note 回调只缓存原始消息和时间戳；所有“当前是否有效”的判断在这里集中计算。
 */
static void update_runtime_check_flags(const ros::Time& now) {
    runtime_flags.vehicle_status_fresh =
        vehicle_status_received && (now - last_vehicle_status_time).toSec() <= vehicle_status_timeout;
    runtime_flags.vehicle_connected = runtime_flags.vehicle_status_fresh && vehicle_status.connected;
    runtime_flags.reliable_vehicle_connection = runtime_flags.vehicle_connected;
    runtime_flags.vehicle_ready = runtime_flags.vehicle_status_fresh && vehicle_status.ready;

    runtime_flags.active_task_status_received = false;
    runtime_flags.active_task_status_fresh = false;
    auto active_it = task_table.find(active_task_id);
    if (active_it != task_table.end()) {
        runtime_flags.active_task_status_received = active_it->second.status_received;
        runtime_flags.active_task_status_fresh =
            active_it->second.status_received &&
            (now - active_it->second.last_status_time).toSec() <= task_status_timeout;
    }
}

/**
 * @brief 根据任务节点缓存状态同步 task_manager 对活跃任务的认知。
 * @param[in] now 当前时间。
 *
 * @note 任务 status callback 不直接改 active_task_id/active_task_run_state，避免输入缓存和管理逻辑耦合。
 */
static void sync_managed_task_state(const ros::Time& now) {
    if (!active_task_id.empty()) {
        auto active_it = task_table.find(active_task_id);
        if (active_it != task_table.end() && active_it->second.status_received) {
            active_task_run_state = task_state_from_string(active_it->second.last_status.task_state);
        }
        return;
    }

    for (const std::string& task_id : registered_task_order) {
        auto it = task_table.find(task_id);
        if (it == task_table.end() || !it->second.status_received) {
            continue;
        }

        bool status_fresh = (now - it->second.last_status_time).toSec() <= task_status_timeout;
        ManagedTaskState incoming_state = task_state_from_string(it->second.last_status.task_state);
        if (status_fresh && task_state_is_busy(incoming_state)) {
            active_task_id = task_id;
            active_task_run_state = incoming_state;
            ROS_INFO("[Task_Manager_node] Adopted active task from status: %s", task_id.c_str());
            return;
        }
    }
}

/**
 * @brief 更新 task_manager 自身状态机。
 * @param[in] now 当前时间。
 */
static void update_node_state(const ros::Time& /*now*/) {
    switch (node_state) {
    case TaskManagerNodeState::INITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_to(TaskManagerNodeState::NORMAL);
        }
        break;

    case TaskManagerNodeState::NORMAL:
        if (!runtime_flags.reliable_vehicle_connection) {
            transition_to(TaskManagerNodeState::REINITIALIZING);
        }
        break;

    case TaskManagerNodeState::REINITIALIZING:
        if (runtime_flags.reliable_vehicle_connection) {
            transition_to(TaskManagerNodeState::NORMAL);
        }
        break;
    }
}

static void refresh_task_manager_context(const ros::Time& now) {
    update_runtime_check_flags(now);
    sync_managed_task_state(now);
    update_node_state(now);
}

/**
 * @brief 判断是否允许启动任务。
 * @param[out] reason 不满足时的原因。
 * @return true=task_manager 和车辆均 ready，可转发任务启动请求。
 *
 * @note TaskManager 不主动请求 OFFBOARD；只读取 connect_manager_node 发布的 ready 状态。
 */
static bool task_start_prerequisites_met(std::string& reason) {
    if (node_state != TaskManagerNodeState::NORMAL) {
        reason = "Task manager is not ready: " + std::string(node_state_to_string(node_state));
        return false;
    }
    if (!runtime_flags.reliable_vehicle_connection) {
        reason = "Vehicle reliable connection unavailable";
        return false;
    }
    if (!runtime_flags.vehicle_ready) {
        reason = "Vehicle is not ready: " + vehicle_status.message;
        return false;
    }
    return true;
}

/**
 * @brief 注册任务节点：创建 start/control service 客户端和 status 订阅。
 * @param[in] nh 节点句柄。
 * @param[in] task_id 任务标识，如 "square_patrol"。
 * @param[in] task_ns 任务命名空间，如 "/uav/tasks/square_patrol"。
 *
 * 每个任务节点必须遵守统一接口：
 *   - start service:  <task_ns>/start   (StartTask)
 *   - control service: <task_ns>/control (ControlTask)
 *   - status topic:   <task_ns>/status  (TaskStatus)
 */
static void register_task(ros::NodeHandle& nh,
                          const std::string& task_id,
                          const std::string& task_ns) {
    TaskEntry entry;
    entry.task_id = task_id;
    entry.task_ns = task_ns;
    entry.status_received = false;
    entry.last_status_time = ros::Time(0);

    // 创建三个接口：start service 客户端 / control service 客户端 / status 订阅
    std::string start_srv_name   = task_ns + "/start";
    std::string control_srv_name = task_ns + "/control";
    std::string status_topic     = task_ns + "/status";

    entry.start_client   = nh.serviceClient<lite_quad_uav::StartTask>(start_srv_name);
    entry.control_client = nh.serviceClient<lite_quad_uav::ControlTask>(control_srv_name);
    entry.status_sub = nh.subscribe<lite_quad_uav::TaskStatus>(status_topic, 10, boost::bind(&task_status_cb, _1, task_id));

    if (task_table.find(task_id) == task_table.end()) {
        registered_task_order.push_back(task_id);
    }
    task_table[task_id] = entry;

    ROS_INFO("[Task_Manager_node] Registered task '%s' (ns=%s)", task_id.c_str(), task_ns.c_str());
}

/**
 * @brief 任务状态回调：只缓存任务节点状态和时间戳。
 * @param[in] msg 任务状态消息。
 * @param[in] task_id 任务标识。
 */
static void task_status_cb(const lite_quad_uav::TaskStatus::ConstPtr& msg,
                            const std::string& task_id) {
    auto it = task_table.find(task_id);
    if (it == task_table.end()) {
        ROS_WARN("[Task_Manager_node] Status received for unregistered task: %s", task_id.c_str());
        return;
    }

    TaskEntry& entry = it->second;
    entry.last_status = *msg;
    entry.status_received = true;
    entry.last_status_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
}

/**
 * @brief 缓存车辆状态，供 start_task ready 门控使用。
 * @param[in] msg connect_manager_node 发布的车辆状态。
 */
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg) {
    vehicle_status = *msg;
    last_vehicle_status_time = ros::Time::now();
    vehicle_status_received = true;
}

static lite_quad_uav::TaskManagerStatus make_task_manager_status(const ros::Time& now) {
    lite_quad_uav::TaskManagerStatus status;
    status.header.stamp = now;

    status.node_state = node_state_to_string(node_state);
    status.busy = task_busy();
    status.accepting_new_task = accepting_new_task();

    status.active_task_id = active_task_id;
    status.active_task_state = active_task_id.empty() ? "IDLE" : task_state_to_string(active_task_run_state);
    status.active_task_progress = "";
    status.active_action = "";
    status.active_task_status_received = runtime_flags.active_task_status_received;
    status.active_task_status_fresh = runtime_flags.active_task_status_fresh;
    status.message = task_manager_message();

    auto active_it = task_table.find(active_task_id);
    if (active_it != task_table.end()) {
        if (active_it->second.status_received) {
            status.active_task_state = active_it->second.last_status.task_state;
            status.active_task_progress = active_it->second.last_status.task_progress;
            status.active_action = active_it->second.last_status.active_action;
        }
    }

    status.registered_task_ids = registered_task_ids();
    status.registered_task_count = static_cast<uint32_t>(task_table.size());

    status.vehicle_status_received = vehicle_status_received;
    status.vehicle_status_fresh = runtime_flags.vehicle_status_fresh;
    status.vehicle_connected = runtime_flags.vehicle_connected;
    status.vehicle_ready = runtime_flags.vehicle_ready;

    return status;
}

/**
 * @brief 周期发布任务管理器聚合状态。
 * @param[in] now 当前时间。
 */
static void publish_task_manager_status(const ros::Time& now) {
    lite_quad_uav::TaskManagerStatus status = make_task_manager_status(now);

    status_pub.publish(status);
}

/**
 * @brief 运行时健康检查定时器：只刷新标志位，不做状态跳转。
 * @param[in] event 定时器事件。
 */
static void runtime_check_timer_cb(const ros::TimerEvent& /*event*/) {
    update_runtime_check_flags(ros::Time::now());
}

/**
 * @brief 周期发布 TaskManagerStatus。
 * @param[in] event 定时器事件。
 */
static void status_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    refresh_task_manager_context(now);
    publish_task_manager_status(now);
}

/**
 * @brief 周期发布任务管理器日志报告，供 logger_node 集中展示。
 * @param[in] event 定时器事件。
 */
static void log_report_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    refresh_task_manager_context(now);
    lite_quad_uav::TaskManagerStatus status = make_task_manager_status(now);

    lite_quad_uav::TaskManagerLogReport report;
    report.header.stamp = now;
    report.node_state = status.node_state;
    report.accepting_new_task = status.accepting_new_task;
    report.active_task_id = active_task_id;
    report.active_task_state = status.active_task_state;
    report.active_task_progress = status.active_task_progress;
    report.active_action = status.active_action;
    report.registered_task_count = static_cast<uint32_t>(task_table.size());
    report.busy = task_busy();

    report.vehicle_status_received = vehicle_status_received;
    report.vehicle_status_fresh = runtime_flags.vehicle_status_fresh;
    report.vehicle_ready = runtime_flags.vehicle_ready;
    report.message = status.message;

    log_report_pub.publish(report);
}

/**
 * @brief 启动任务服务回调：校验后转发 start 请求到任务节点。
 * @param[in] req 启动请求。
 * @param[out] res 启动响应。
 * @return true=服务调用成功。
 */
static bool start_task_cb(lite_quad_uav::StartTask::Request& req,
                           lite_quad_uav::StartTask::Response& res) {
    ros::Time now = ros::Time::now();
    refresh_task_manager_context(now);

    // Step 1: 互斥检查 — 已有任务在运行时拒绝新启动
    if (!active_task_id.empty() && task_busy()) {
        res.accepted = false;
        res.message = "Another task is active: " + active_task_id;
        return true;
    }

    // Step 2: 查找任务 — 确认 task_type 已注册
    auto it = task_table.find(req.task_type);
    if (it == task_table.end()) {
        res.accepted = false;
        res.message = "Unknown task type: " + req.task_type;
        return true;
    }

    TaskEntry& entry = it->second;

    // Step 3: 车辆 ready 门控 — OFFBOARD 准备由 connect_manager_node 启动后自动完成
    std::string reason;
    if (!task_start_prerequisites_met(reason)) {
        res.accepted = false;
        res.message = reason;
        return true;
    }

    // Step 4: 转发 start 请求到任务节点 service
    lite_quad_uav::StartTask srv;
    srv.request.task_type = req.task_type;
    srv.request.task_params_json = req.task_params_json;
    if (!entry.start_client.call(srv)) {
        ROS_WARN("[Task_Manager_node] Failed to call start service for task: %s", req.task_type.c_str());
        res.accepted = false;
        res.message = "Failed to call start service for task: " + req.task_type;
        return true;
    }

    // Step 5: 采纳 — 若任务节点接受，记录 active_task_id 并设置 active_task_run_state=RUNNING
    res.accepted = srv.response.accepted;
    res.message = srv.response.message;
    if (res.accepted) {
        active_task_id = req.task_type;
        active_task_run_state = ManagedTaskState::RUNNING;
        ROS_INFO("[Task_Manager_node] Started task: %s", req.task_type.c_str());
    }

    return true;
}

/**
 * @brief 控制任务服务回调：转发 PAUSE/RESUME/CANCEL 命令到活跃任务节点。
 * @param[in] req 控制请求。
 * @param[out] res 控制响应。
 * @return true=服务调用成功。
 *
 * @note CANCELING 状态需等待任务节点回报 CANCELED/FAILED 终态。
 */
static bool control_task_cb(lite_quad_uav::ControlTask::Request& req,
                             lite_quad_uav::ControlTask::Response& res) {
    ros::Time now = ros::Time::now();
    refresh_task_manager_context(now);

    // Step 1: 解析命令 — 不识别的命令直接拒绝
    TaskManagerCommand command = parse_control_command(req.command);

    if (command == TaskManagerCommand::UNKNOWN) {
        res.accepted = false;
        res.message = "Unsupported command: " + req.command;
        return true;
    }

    // Step 2: 确定目标任务 — 优先使用请求中的 task_id，否则回退到 active_task_id
    std::string target_task_id = req.task_id.empty() ? active_task_id : req.task_id;

    if (target_task_id.empty()) {
        res.accepted = false;
        res.message = "No task specified and no active task";
        return true;
    }

    // Step 3: 查找任务 — 确认目标任务已注册
    auto it = task_table.find(target_task_id);
    if (it == task_table.end()) {
        res.accepted = false;
        res.message = "Unknown task: " + target_task_id;
        return true;
    }

    // Step 4: 校验活跃性 — 只能控制当前活跃任务，不允许操作非活跃任务
    if (target_task_id != active_task_id) {
        res.accepted = false;
        res.message = "Task '" + target_task_id + "' is not the active task";
        return true;
    }

    // Step 5: 校验可控制性 — 终态任务不可再控制
    if (active_task_run_state == ManagedTaskState::SUCCEEDED ||
        active_task_run_state == ManagedTaskState::CANCELED ||
        active_task_run_state == ManagedTaskState::FAILED) {
        res.accepted = false;
        res.message = "Active task is not running";
        return true;
    }

    TaskEntry& entry = it->second;

    // Step 6: 状态机门控 — 重连等待期间仍允许 CANCEL，拒绝 PAUSE/RESUME
    if (command != TaskManagerCommand::CANCEL && node_state != TaskManagerNodeState::NORMAL) {
        res.accepted = false;
        res.message = "Task manager is not ready: " + std::string(node_state_to_string(node_state));
        return true;
    }

    // Step 7: 转发控制命令到任务节点 service
    lite_quad_uav::ControlTask srv;
    srv.request.task_id = target_task_id;
    switch (command) {
    case TaskManagerCommand::PAUSE:  srv.request.command = "PAUSE";  break;
    case TaskManagerCommand::RESUME: srv.request.command = "RESUME"; break;
    case TaskManagerCommand::CANCEL: srv.request.command = "CANCEL"; break;
    default:                         srv.request.command = req.command; break;
    }
    if (!entry.control_client.call(srv)) {
        ROS_WARN("[Task_Manager_node] Failed to call control service for task: %s", target_task_id.c_str());
        res.accepted = false;
        res.message = "Failed to call control service for task: " + target_task_id;
        return true;
    }

    // Step 8: 同步 active_task_run_state — 根据命令类型更新管理状态
    res.accepted = srv.response.accepted;
    res.message = srv.response.message;
    if (res.accepted) {
        switch (command) {
        case TaskManagerCommand::PAUSE:  active_task_run_state = ManagedTaskState::PAUSED;    break;
        case TaskManagerCommand::RESUME: active_task_run_state = ManagedTaskState::RUNNING;   break;
        case TaskManagerCommand::CANCEL: active_task_run_state = ManagedTaskState::CANCELING; break;
        default: break;
        }
        ROS_INFO("[Task_Manager_node] Control command '%s' accepted for task: %s",
                 req.command.c_str(), target_task_id.c_str());
    }

    return true;
}
