/**
 * @file connect_manager_node.cpp
 * @brief PX4/MAVROS 连接管理节点：确认飞控稳定连接，并代理飞控模式/解锁命令。
 *
 * 本节点只负责连接和飞控模式准备，不作为常规 setpoint 出口。
 * 常规速度 setpoint 由 velocity_manager_node 发布，避免多个节点同时写 MAVROS 控制话题。
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <algorithm>
#include <cctype>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <lite_quad_uav/PipelineStatus.h>
#include <lite_quad_uav/ConnectManagerLogReport.h>
#include <lite_quad_uav/VehicleCommand.h>
#include <lite_quad_uav/VehicleStatus.h>

/* Private typedef / 私有类型定义 --------------------------------------------*/
enum VehicleState {
    INIT_WAIT_MAVROS,              // 初始化阶段：等待 MAVROS 服务/接口可用

    LINK_WAIT_FCU_CONNECTION,      // 飞控连接阶段：等待 FCU connected
    LINK_WAIT_LOCAL_POSITION,      // 飞控连接阶段：等待 local_position/pose 稳定接收

    STREAM_WAIT_SETPOINT_STREAM,   // setpoint 前置阶段：等待 velocity_manager_node 持续发布 setpoint
    COMMAND_READY,                 // 命令待机：可接收 SET_MODE / ARM / DISARM

    MODE_REQUEST,                  // 模式控制：发送 set_mode 请求
    MODE_WAIT_CONFIRM,             // 模式控制：等待 MAVROS state 确认目标模式

    ARM_CTRL_REQUEST,              // 解锁/上锁控制：发送 arming 请求
    ARM_CTRL_WAIT_CONFIRM,         // 解锁/上锁控制：等待 armed 状态确认

    RECOVERY                       // 异常恢复阶段：连接/定位/setpoint 异常后的降级恢复入口
};

struct RuntimeCheckFlags {
    bool mavros_services_ready = false;
    bool mavros_state_fresh = false;
    bool fcu_connected = false;
    bool local_position_fresh = false;
    bool velocity_manager_alive = false;
    bool reliable_connection = false;
    bool offboard_confirmed = false;
};

/* Private variables / 私有变量 ----------------------------------------------*/

static VehicleState vehicle_state = INIT_WAIT_MAVROS;
static ros::Time state_enter_time;
static ros::Time last_mode_request_time;

static std::string mavros_ns = "/mavros";
static std::string requested_mode;
static bool requested_arm_state = false;
static double connect_timeout = 30.0;
static double position_timeout = 5.0;
static double setpoint_stream_timeout = 10.0;
static double mode_retry_interval = 2.0;
static double mode_confirm_timeout = 3.0;
static double state_timeout = 3.0;
static double runtime_check_rate = 20.0;
static double status_publish_rate = 2.0;
static double log_report_rate = 1.0;
static double main_loop_rate = 50.0;

static mavros_msgs::State mavros_state;
static bool state_received = false;
static ros::Time last_state_time;

static geometry_msgs::PoseStamped last_pose;
static bool local_position_received = false;
static ros::Time last_pose_time;

static lite_quad_uav::PipelineStatus velocity_manager_status;
static bool velocity_manager_status_received = false;
static ros::Time last_velocity_manager_status_time;
static RuntimeCheckFlags runtime_flags;
static std::string last_vehicle_command;
static bool last_vehicle_command_success = false;
static std::string last_vehicle_command_result;
static ros::Time last_vehicle_command_time;

static ros::Publisher status_pub;
static ros::Publisher log_report_pub;
static ros::ServiceClient set_mode_client;
static ros::ServiceClient arming_client;

/* Private function prototypes / 私有函数声明 --------------------------------*/

static const char* vehicle_state_to_string(VehicleState state);
static const char* vehicle_state_message(VehicleState state);
static void transition_to(VehicleState next_state);
static std::string normalize_command(const std::string& command);
static std::string trim_copy(const std::string& text);
static void record_vehicle_command_result(bool success, const std::string& result);
static bool velocity_manager_streaming_state();
static void update_runtime_check_flags(const ros::Time& now);
static bool vehicle_ready_for_command();
static bool vehicle_command_busy();
static bool request_mode(const std::string& mode);
static bool request_arm(bool arm, std::string& message);
static void handle_connection_lost_or_timeout(const std::string& reason);
static bool check_runtime_faults();
static void state_cb(const mavros_msgs::State::ConstPtr& msg);
static void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
static void velocity_manager_status_cb(const lite_quad_uav::PipelineStatus::ConstPtr& msg);
static void runtime_check_timer_cb(const ros::TimerEvent& event);
static void status_timer_cb(const ros::TimerEvent& event);
static void log_report_timer_cb(const ros::TimerEvent& event);
static bool vehicle_command_cb(lite_quad_uav::VehicleCommand::Request& req, lite_quad_uav::VehicleCommand::Response& res);

/* Main function / 主函数 -----------------------------------------------------*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "connect_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 运行参数只影响等待超时、发布频率和调试 setpoint，不改变对外接口。
    pnh.param<std::string>("mavros_ns", mavros_ns, "/mavros");
    pnh.param<double>("connect_timeout", connect_timeout, 30.0);
    pnh.param<double>("position_timeout", position_timeout, 5.0);
    pnh.param<double>("setpoint_stream_timeout", setpoint_stream_timeout, 10.0);
    pnh.param<double>("mode_retry_interval", mode_retry_interval, 2.0);
    pnh.param<double>("mode_confirm_timeout", mode_confirm_timeout, 3.0);
    pnh.param<double>("state_timeout", state_timeout, 3.0);
    pnh.param<double>("runtime_check_rate", runtime_check_rate, 20.0);
    pnh.param<double>("status_publish_rate", status_publish_rate, 2.0);
    pnh.param<double>("log_report_rate", log_report_rate, 1.0);
    pnh.param<double>("main_loop_rate", main_loop_rate, 10.0);

    if (state_timeout <= 0.0) {
        ROS_WARN("[Connect_Manager_node] state_timeout must be positive, reset to 3.0");
        state_timeout = 3.0;
    }
    if (mode_retry_interval <= 0.0) {
        ROS_WARN("[Connect_Manager_node] mode_retry_interval must be positive, reset to 2.0");
        mode_retry_interval = 2.0;
    }
    if (mode_confirm_timeout <= 0.0) {
        ROS_WARN("[Connect_Manager_node] mode_confirm_timeout must be positive, reset to 3.0");
        mode_confirm_timeout = 3.0;
    }
    if (runtime_check_rate <= 0.0) {
        ROS_WARN("[Connect_Manager_node] runtime_check_rate must be positive, reset to 20.0");
        runtime_check_rate = 20.0;
    }
    if (status_publish_rate <= 0.0) {
        ROS_WARN("[Connect_Manager_node] status_publish_rate must be positive, reset to 2.0");
        status_publish_rate = 2.0;
    }
    if (log_report_rate <= 0.0) {
        ROS_WARN("[Connect_Manager_node] log_report_rate must be positive, reset to 1.0");
        log_report_rate = 1.0;
    }
    if (main_loop_rate <= 0.0) {
        ROS_WARN("[Connect_Manager_node] main_loop_rate must be positive, reset to 10.0");
        main_loop_rate = 10.0;
    }

    // ── 话题与服务名称 ──────────────────────────────────────────────────
    const std::string velocity_manager_status_topic = "/uav/control/velocity_manager_status";  // 速度管理管道状态 (判断 setpoint 流是否就绪)
    const std::string vehicle_status_topic = "/uav/vehicle/status";           // 对外车辆连接状态 (周期发布)
    const std::string log_report_topic = "/uav/log_reports/connect_manager";  // 节点日志报告 (logger_node 消费)
    const std::string vehicle_command_srv_name = "/uav/vehicle/command";      // 车辆命令服务 (task 层触发 SET_MODE/ARM)

    // 输入回调只缓存 MAVROS、local position 和速度管理层状态，主循环统一做流程判断。
    ros::Subscriber state_sub = nh.subscribe(mavros_ns + "/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe(mavros_ns + "/local_position/pose", 10, local_pos_cb);
    ros::Subscriber velocity_manager_status_sub = nh.subscribe(velocity_manager_status_topic, 10, velocity_manager_status_cb);

    // 对外状态用于通知其他节点连接和 setpoint stream 是否满足命令前置条件。
    status_pub = nh.advertise<lite_quad_uav::VehicleStatus>(vehicle_status_topic, 10);
    log_report_pub = nh.advertise<lite_quad_uav::ConnectManagerLogReport>(log_report_topic, 10);

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(mavros_ns + "/cmd/arming");
    // task 层通过该服务触发 PREPARE / SET_MODE / ARM / DISARM。
    ros::ServiceServer vehicle_command_srv = nh.advertiseService(vehicle_command_srv_name, vehicle_command_cb);

    // 状态发布使用定时器，避免主状态机里混入固定频率发布逻辑。
    ros::Timer runtime_check_timer = nh.createTimer(ros::Duration(1.0 / runtime_check_rate), runtime_check_timer_cb);
    ros::Timer status_timer = nh.createTimer(ros::Duration(1.0 / status_publish_rate), status_timer_cb);
    ros::Timer log_report_timer = nh.createTimer(ros::Duration(1.0 / log_report_rate), log_report_timer_cb);

    state_enter_time = ros::Time::now();
    ROS_INFO("[Connect_Manager_node] Initialized, mavros_ns = %s", mavros_ns.c_str());

    ros::Rate rate(main_loop_rate);
    while (ros::ok()) {
        ros::spinOnce();
        ros::Time now = ros::Time::now();
        update_runtime_check_flags(now);

        // 准备完成后若连接、定位或 setpoint 丢失，先降级恢复再继续状态机。
        if (check_runtime_faults()) {
            rate.sleep();
            continue;
        }

        // ── 主状态机 ──────────────────────────────────────────────────────────
        // 流程：MAVROS 可用 → FCU 连接 → 定位有效 → 速度 setpoint 流有效 → 命令待机
        // runtime_flags 由定时器和主循环统一刷新，状态机只做阶段跳转。
        switch (vehicle_state) {

        /* ── 阶段 1: 基础连通性检查 ─────────────────────────────────────────── */

        case INIT_WAIT_MAVROS:
            // 条件：set_mode + arming 两个 MAVROS service 都已注册
            // 超时 → RECOVERY；未就绪 → 持续等待
            if (runtime_flags.mavros_services_ready) {
                ROS_INFO("[Connect_Manager_node] MAVROS command services are available");
                transition_to(LINK_WAIT_FCU_CONNECTION);
            } else if ((now - state_enter_time).toSec() > connect_timeout) {
                handle_connection_lost_or_timeout("MAVROS services timeout");
            } else {
                ROS_INFO_THROTTLE(5.0, "[Connect_Manager_node] Waiting for MAVROS services...");
            }
            break;

        case LINK_WAIT_FCU_CONNECTION:
            // 条件：MAVROS state 新鲜且 connected=true，即 MAVROS 已收到 FCU 心跳
            // 心跳存在不代表数据链路可靠，还需要 LINK_WAIT_LOCAL_POSITION 进一步确认
            if (runtime_flags.fcu_connected) {
                ROS_INFO("[Connect_Manager_node] FCU heartbeat received, current mode = %s", mavros_state.mode.c_str());
                transition_to(LINK_WAIT_LOCAL_POSITION);
            } else if ((now - state_enter_time).toSec() > connect_timeout) {
                handle_connection_lost_or_timeout("FCU connection timeout");
            } else {
                ROS_INFO_THROTTLE(5.0, "[Connect_Manager_node] Waiting for FCU connection...");
            }
            break;

        case LINK_WAIT_LOCAL_POSITION:
            // 条件：收到新鲜的 local_position/pose，认为 MAVROS↔FCU 数据链路已可靠
            if (runtime_flags.local_position_fresh) {
                ROS_INFO("[Connect_Manager_node] Reliable FCU connection established");
                transition_to(STREAM_WAIT_SETPOINT_STREAM);
            } else if ((now - state_enter_time).toSec() > connect_timeout) {
                handle_connection_lost_or_timeout("local_position/pose timeout before stable connection");
            } else {
                ROS_INFO_THROTTLE(5.0, "[Connect_Manager_node] Waiting for local_position/pose...");
            }
            break;

        /* ── 阶段 2: 速度管理层 setpoint 前置条件 ─────────────────────────── */

        case STREAM_WAIT_SETPOINT_STREAM:
            // 条件：velocity_manager 已进入流式发布状态 (SAFE_HOLD/NORMAL/SAFETY_FILTER)
            // PX4 OFFBOARD 模式要求 setpoint 持续输入，所以必须等 velocity_manager 先启动
            if (runtime_flags.velocity_manager_alive) {
                ROS_INFO("[Connect_Manager_node] Checked velocity manager is running, state = %s", velocity_manager_status.state.c_str());
                transition_to(COMMAND_READY);
            } else if ((now - state_enter_time).toSec() > setpoint_stream_timeout) {
                handle_connection_lost_or_timeout("velocity manager timeout, retry waiting for setpoint stream");
            } else {
                ROS_INFO_THROTTLE(5.0, "[Connect_Manager_node] Waiting for velocity manager running and setpoint stream...");
            }
            break;

        /* ── 阶段 3: 命令待机 / 模式控制 ───────────────────────────────────── */

        case COMMAND_READY:
            ROS_INFO_THROTTLE(10.0, "[Connect_Manager_node] Command ready, waiting for SET_MODE / ARM / DISARM");
            break;

        case MODE_REQUEST:
            // 按间隔发送 set_mode 请求，避免频繁调用 MAVROS service。
            if ((last_mode_request_time == ros::Time(0)) || (now - last_mode_request_time).toSec() >= mode_retry_interval) {
                last_mode_request_time = now;
                if (mavros_state.mode == requested_mode) {
                    ROS_INFO("[Connect_Manager_node] Mode already confirmed: %s", requested_mode.c_str());
                    transition_to(COMMAND_READY);
                } else if (request_mode(requested_mode)) {
                    ROS_INFO("[Connect_Manager_node] Set mode request sent: %s", requested_mode.c_str());
                    transition_to(MODE_WAIT_CONFIRM);
                }
            }
            break;

        case MODE_WAIT_CONFIRM:
            if (mavros_state.mode == requested_mode) {
                ROS_INFO("[Connect_Manager_node] Mode confirmed: %s", requested_mode.c_str());
                transition_to(COMMAND_READY);
            } else if ((now - state_enter_time).toSec() > mode_confirm_timeout) {
                ROS_WARN("[Connect_Manager_node] Mode confirmation timeout for %s, retrying", requested_mode.c_str());
                transition_to(MODE_REQUEST);
            } else {
                ROS_INFO_THROTTLE(2.0, "[Connect_Manager_node] Waiting for mode confirmation, target=%s current=%s",
                                  requested_mode.c_str(), mavros_state.mode.c_str());
            }
            break;

        /* ── 阶段 4: 解锁 / 上锁控制 ─────────────────────────────────────── */

        case ARM_CTRL_REQUEST: {
            std::string message;
            if (mavros_state.armed == requested_arm_state) {
                ROS_INFO("[Connect_Manager_node] Arm state already confirmed: %s", requested_arm_state ? "armed" : "disarmed");
                transition_to(COMMAND_READY);
            } else if (request_arm(requested_arm_state, message)) {
                ROS_INFO("[Connect_Manager_node] Arm control request sent, waiting for state: %s",
                         requested_arm_state ? "armed" : "disarmed");
                transition_to(ARM_CTRL_WAIT_CONFIRM);
            } else {
                ROS_WARN("[Connect_Manager_node] %s", message.c_str());
                transition_to(COMMAND_READY);
            }
            break;
        }

        case ARM_CTRL_WAIT_CONFIRM:
            if (mavros_state.armed == requested_arm_state) {
                ROS_INFO("[Connect_Manager_node] Arm state confirmed: %s", requested_arm_state ? "armed" : "disarmed");
                transition_to(COMMAND_READY);
            } else if ((now - state_enter_time).toSec() > connect_timeout) {
                ROS_WARN("[Connect_Manager_node] Arm control confirmation timeout");
                transition_to(COMMAND_READY);
            }
            break;

        /* ── 异常恢复 ─────────────────────────────────────────────────────── */

        case RECOVERY:
            // 按优先级从高到低逐级检查，跳转到第一个未满足条件的状态重新等待：
            // MAVROS 服务 → FCU 连接 → 定位 → 速度管理 → 命令待机
            if (!runtime_flags.mavros_services_ready) {
                transition_to(INIT_WAIT_MAVROS);
            } else if (!runtime_flags.fcu_connected) {
                transition_to(LINK_WAIT_FCU_CONNECTION);
            } else if (!runtime_flags.local_position_fresh) {
                transition_to(LINK_WAIT_LOCAL_POSITION);
            } else if (!runtime_flags.velocity_manager_alive) {
                transition_to(STREAM_WAIT_SETPOINT_STREAM);
            } else {
                transition_to(COMMAND_READY);
            }
            break;
        }

        rate.sleep();
    }

    return 0;
}

/* Private function implementations / 私有函数实现 ----------------------------*/
/**
 * @brief 将连接管理状态枚举转为字符串。
 * @param[in] state 状态枚举值。
 * @return 状态名称字符串。
 */
static const char* vehicle_state_to_string(VehicleState state) {
    switch (state) {
    case INIT_WAIT_MAVROS:              return "INIT_WAIT_MAVROS";
    case LINK_WAIT_FCU_CONNECTION:      return "LINK_WAIT_FCU_CONNECTION";
    case LINK_WAIT_LOCAL_POSITION:      return "LINK_WAIT_LOCAL_POSITION";
    case STREAM_WAIT_SETPOINT_STREAM:   return "STREAM_WAIT_SETPOINT_STREAM";
    case COMMAND_READY:                 return "COMMAND_READY";
    case MODE_REQUEST:                  return "MODE_REQUEST";
    case MODE_WAIT_CONFIRM:             return "MODE_WAIT_CONFIRM";
    case ARM_CTRL_REQUEST:              return "ARM_CTRL_REQUEST";
    case ARM_CTRL_WAIT_CONFIRM:         return "ARM_CTRL_WAIT_CONFIRM";
    case RECOVERY:                      return "RECOVERY";
    }
    return "UNKNOWN";
}

/**
 * @brief 将连接管理状态转为面向日志/状态话题的人可读描述。
 * @param[in] state 状态枚举值。
 * @return 状态描述字符串。
 */
static const char* vehicle_state_message(VehicleState state) {
    switch (state) {
    case INIT_WAIT_MAVROS:              return "Waiting for MAVROS services";
    case LINK_WAIT_FCU_CONNECTION:      return "Waiting for FCU connection";
    case LINK_WAIT_LOCAL_POSITION:      return "Waiting for local position";
    case STREAM_WAIT_SETPOINT_STREAM:   return "Waiting for velocity manager setpoint stream";
    case COMMAND_READY:                 return "Ready for vehicle command";
    case MODE_REQUEST:                  return "Requesting flight mode";
    case MODE_WAIT_CONFIRM:             return "Waiting for flight mode confirmation";
    case ARM_CTRL_REQUEST:              return "Requesting arm control";
    case ARM_CTRL_WAIT_CONFIRM:         return "Waiting for arm control confirmation";
    case RECOVERY:                      return "Recovering connection state";
    }
    return "Unknown vehicle state";
}

/**
 * @brief 切换到下一状态，记录进入时间。
 * @param[in] next_state 目标状态。
 *
 * @note 进入 MODE_REQUEST 时重置 last_mode_request_time=0，
 *       使首次模式请求在主循环中立即发出而非等待 mode_retry_interval。
 */
static void transition_to(VehicleState next_state) {
    vehicle_state = next_state;
    state_enter_time = ros::Time::now();

    if (next_state == MODE_REQUEST) {
        last_mode_request_time = ros::Time(0);
    }
}

/**
 * @brief 命令字符串转大写，用于不区分大小写的命令匹配。
 * @param[in] command 原始命令字符串。
 * @return 大写化后的命令。
 */
static std::string normalize_command(const std::string& command) {
    std::string normalized = command;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) { return std::toupper(c); });
    return normalized;
}

/**
 * @brief 去除字符串首尾空白字符。
 * @param[in] text 原始字符串。
 * @return 去除首尾空白后的字符串。
 */
static std::string trim_copy(const std::string& text) {
    std::string::size_type first = 0;
    while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first]))) {
        ++first;
    }

    std::string::size_type last = text.size();
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) {
        --last;
    }

    return text.substr(first, last - first);
}

/**
 * @brief 缓存最近一次车辆命令执行结果，供 log report 输出。
 * @param[in] success 命令是否被接受/执行成功。
 * @param[in] result 命令结果描述。
 */
static void record_vehicle_command_result(bool success, const std::string& result) {
    last_vehicle_command_success = success;
    last_vehicle_command_result = result;
    last_vehicle_command_time = ros::Time::now();
}

/**
 * @brief 检查 velocity_manager 状态是否表示 MAVROS setpoint 正在流式发布。
 * @return true=速度管理层处于可作为 OFFBOARD 前置条件的流式状态。
 */
static bool velocity_manager_streaming_state() {
    return velocity_manager_status.state == "SAFE_HOLD" ||
           velocity_manager_status.state == "NORMAL" ||
           velocity_manager_status.state == "SAFETY_FILTER";
}

/**
 * @brief 统一刷新运行时健康标志。
 * @param[in] now 当前时间。
 *
 * @note 回调只缓存原始数据和时间戳；所有“当前是否有效”的判断在这里集中计算。
 */
static void update_runtime_check_flags(const ros::Time& now) {
    runtime_flags.mavros_services_ready = set_mode_client.exists() && arming_client.exists();
    runtime_flags.mavros_state_fresh = state_received && (now - last_state_time).toSec() <= state_timeout;
    runtime_flags.fcu_connected = runtime_flags.mavros_state_fresh && mavros_state.connected;
    runtime_flags.local_position_fresh = local_position_received && (now - last_pose_time).toSec() <= position_timeout;

    bool velocity_manager_status_fresh = velocity_manager_status_received && (now - last_velocity_manager_status_time).toSec() <= setpoint_stream_timeout;
    runtime_flags.velocity_manager_alive = velocity_manager_status_fresh && velocity_manager_streaming_state();

    runtime_flags.reliable_connection = runtime_flags.fcu_connected && runtime_flags.local_position_fresh;
    runtime_flags.offboard_confirmed = runtime_flags.mavros_state_fresh && mavros_state.mode == "OFFBOARD";
}

/**
 * @brief 判断系统是否满足车辆命令前置条件。
 * @return true=系统就绪。
 *
 * @note VehicleStatus.ready=true 表示可靠连接和 velocity_manager setpoint stream 已满足，
 *       可由任务节点请求 SET_MODE/ARM，不表示当前已经是 OFFBOARD 或已解锁。
 */
static bool vehicle_ready_for_command() {
    return runtime_flags.fcu_connected &&
           runtime_flags.local_position_fresh &&
           runtime_flags.velocity_manager_alive;
}

/**
 * @brief 判断当前是否已有车辆命令正在执行。
 * @return true=模式或上锁/解锁命令正在等待请求/确认。
 */
static bool vehicle_command_busy() {
    return vehicle_state == MODE_REQUEST ||
           vehicle_state == MODE_WAIT_CONFIRM ||
           vehicle_state == ARM_CTRL_REQUEST ||
           vehicle_state == ARM_CTRL_WAIT_CONFIRM;
}

/**
 * @brief 发送飞控模式切换请求。
 * @param[in] mode PX4 目标模式。
 * @return true=MAVROS service 调用成功且 mode_sent=true。
 *
 * @note 此处只发送请求，是否真正生效由 MODE_WAIT_CONFIRM 状态通过 mavros_state.mode 再确认。
 */
static bool request_mode(const std::string& mode) {
    mavros_msgs::SetMode set_mode;
    set_mode.request.base_mode = 0;
    set_mode.request.custom_mode = mode;

    if (!set_mode_client.call(set_mode)) {
        ROS_WARN_THROTTLE(2.0, "[Connect_Manager_node] set_mode service call failed");
        return false;
    }
    if (!set_mode.response.mode_sent) {
        ROS_WARN_THROTTLE(2.0, "[Connect_Manager_node] set_mode returned mode_sent=false");
        return false;
    }

    return true;
}

/**
 * @brief 发送解锁/上锁命令。
 * @param[in] arm true=解锁，false=上锁。
 * @param[out] message 命令结果描述。
 * @return true=MAVROS service 调用成功且 FCU 接受命令。
 *
 * @note 解锁/上锁只响应任务命令，不在连接准备流程中自动触发。
 */
static bool request_arm(bool arm, std::string& message) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;

    if (!arming_client.call(arm_cmd)) {
        message = arm ? "Arm service call failed" : "Disarm service call failed";
        return false;
    }
    if (!arm_cmd.response.success) {
        message = arm ? "Arm denied by FCU" : "Disarm denied by FCU";
        return false;
    }

    message = arm ? "Arm command accepted" : "Disarm command accepted";
    return true;
}

/**
 * @brief 处理连接丢失或超时：降级到 RECOVERY 状态重新等待。
 * @param[in] reason 降级原因描述。
 *
 * @note 当前恢复策略只降级并重新等待，不重启 MAVROS，也不执行系统命令。
 */
static void handle_connection_lost_or_timeout(const std::string& reason) {
    ROS_WARN_THROTTLE(2.0, "[Connect_Manager_node] Recovery required: %s", reason.c_str());
    transition_to(RECOVERY);
}

/**
 * @brief 运行时故障检测：命令待机和命令执行阶段检查连接、定位和 setpoint 是否仍正常。
 * @return true=检测到故障并已触发 RECOVERY，主循环应跳过状态机；false=无故障。
 *
 * @note 早期等待状态本身就是恢复过程，不重复触发 RECOVERY。
 */
static bool check_runtime_faults() {
    if (vehicle_state == INIT_WAIT_MAVROS ||
        vehicle_state == LINK_WAIT_FCU_CONNECTION ||
        vehicle_state == LINK_WAIT_LOCAL_POSITION ||
        vehicle_state == STREAM_WAIT_SETPOINT_STREAM ||
        vehicle_state == RECOVERY) {
        return false;
    }

    // 故障检查顺序（与 RECOVERY 恢复优先级一致）：
    if (!runtime_flags.fcu_connected) {
        handle_connection_lost_or_timeout("FCU disconnected");
        return true;
    }
    if (!runtime_flags.local_position_fresh) {
        handle_connection_lost_or_timeout("local_position/pose timeout");
        return true;
    }
    if (!runtime_flags.velocity_manager_alive) {
        handle_connection_lost_or_timeout("velocity manager timeout");
        return true;
    }
    return false;
}

/**
 * @brief MAVROS 飞控状态回调：缓存状态并检测 mode/armed 变化输出 DEBUG 日志。
 * @param[in] msg MAVROS state 消息。
 */
static void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    mavros_msgs::State prev = mavros_state;
    mavros_state = *msg;
    last_state_time = ros::Time::now();
    state_received = true;

    // 检测 mode/armed 变化并输出 DEBUG 日志，便于排查模式切换时序
    if (prev.mode != msg->mode) {
        ROS_DEBUG("[Connect_Manager_node] Mode changed: %s -> %s", prev.mode.c_str(), msg->mode.c_str());
    }
    if (prev.armed != msg->armed) {
        ROS_DEBUG("[Connect_Manager_node] Armed state changed: %d -> %d", prev.armed, msg->armed);
    }
}

/**
 * @brief 本地位置回调：缓存位置数据并刷新时间戳。
 * @param[in] msg 本地位置消息。
 */
static void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_pose = *msg;
    last_pose_time = ros::Time::now();
    local_position_received = true;
}

/**
 * @brief 速度管理管道状态回调：缓存管道状态，健康标志由定时器统一计算。
 * @param[in] msg 管道状态消息。
 */
static void velocity_manager_status_cb(const lite_quad_uav::PipelineStatus::ConstPtr& msg) {
    velocity_manager_status = *msg;
    last_velocity_manager_status_time = ros::Time::now();
    velocity_manager_status_received = true;
}

/**
 * @brief 运行时健康检查定时器：只刷新标志位，不做状态跳转。
 * @param[in] event 定时器事件。
 */
static void runtime_check_timer_cb(const ros::TimerEvent& /*event*/) {
    update_runtime_check_flags(ros::Time::now());
}

/**
 * @brief 周期发布 VehicleStatus 供其他节点判断系统就绪状态。
 * @param[in] event 定时器事件。
 */
static void status_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    update_runtime_check_flags(now);

    lite_quad_uav::VehicleStatus status;
    status.header.stamp = now;

    // connected 表示 state 新鲜、FCU connected 且 local_position 新鲜。
    status.connected = runtime_flags.reliable_connection;
    status.ready = vehicle_ready_for_command();
    status.mode = mavros_state.mode;
    status.armed = mavros_state.armed;

    // ── 状态描述：按当前 vehicle_state 生成人可读消息
    status.message = vehicle_state_message(vehicle_state);

    status_pub.publish(status);
}

/**
 * @brief 周期发布连接管理节点日志报告，供 logger_node 集中展示。
 * @param[in] event 定时器事件。
 */
static void log_report_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();
    update_runtime_check_flags(now);

    lite_quad_uav::ConnectManagerLogReport report;
    report.header.stamp = now;
    report.state = vehicle_state_to_string(vehicle_state);
    report.state_age_sec = (now - state_enter_time).toSec();

    report.mavros_services_ready = runtime_flags.mavros_services_ready;
    report.mavros_state_fresh = runtime_flags.mavros_state_fresh;
    report.fcu_connected = runtime_flags.fcu_connected;
    report.local_position_fresh = runtime_flags.local_position_fresh;
    report.velocity_manager_alive = runtime_flags.velocity_manager_alive;
    report.reliable_connection = runtime_flags.reliable_connection;
    report.offboard_confirmed = runtime_flags.offboard_confirmed;

    report.armed = mavros_state.armed;
    report.ready = vehicle_ready_for_command();
    report.mode = mavros_state.mode;

    report.last_command = last_vehicle_command;
    report.last_command_success = last_vehicle_command_success;
    report.last_command_result = last_vehicle_command_result;
    report.message = vehicle_state_message(vehicle_state);

    log_report_pub.publish(report);
}

/**
 * @brief 车辆命令服务回调：处理 PREPARE / SET_MODE / ARM / DISARM 命令。
 * @param[in] req 车辆命令请求。
 * @param[out] res 命令执行结果。
 * @return true=服务调用成功。
 */
static bool vehicle_command_cb(lite_quad_uav::VehicleCommand::Request& req, lite_quad_uav::VehicleCommand::Response& res) {
    std::string command = normalize_command(trim_copy(req.command));
    ros::Time now = ros::Time::now();
    update_runtime_check_flags(now);
    last_vehicle_command = command;
    last_vehicle_command_time = now;
    last_vehicle_command_success = false;
    last_vehicle_command_result = "Processing";

    ROS_INFO("[Connect_Manager_node] Vehicle command received: %s", command.c_str());

    /* ── PREPARE: 重置到初始状态，重新走连接准备流程 ── */
    if (command == "PREPARE") {
        transition_to(INIT_WAIT_MAVROS);
        res.success = true;
        res.message = "Prepare sequence requested";
        record_vehicle_command_result(res.success, res.message);
        return true;
    }

    /* ── SET_MODE: 只接受请求并切入状态机，实际 set_mode 由 MODE_REQUEST 发送 ── */
    if (command == "SET_MODE") {
        std::string target_mode = trim_copy(req.param);
        if (target_mode.empty()) {
            res.success = false;
            res.message = "SET_MODE param is empty";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }
        if (!runtime_flags.mavros_services_ready) {
            res.success = false;
            res.message = "MAVROS services unavailable";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }
        if (!runtime_flags.fcu_connected) {
            res.success = false;
            res.message = "FCU not connected";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }
        if (!runtime_flags.local_position_fresh) {
            res.success = false;
            res.message = "Local position not ready";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }
        if (!runtime_flags.velocity_manager_alive) {
            res.success = false;
            res.message = "Velocity manager setpoint stream not ready";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }
        if (vehicle_command_busy()) {
            res.success = false;
            res.message = "Vehicle command is busy";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }

        requested_mode = target_mode;
        if (mavros_state.mode == requested_mode) {
            res.success = true;
            res.message = "Mode already active: " + requested_mode;
            record_vehicle_command_result(res.success, res.message);
            return true;
        }

        transition_to(MODE_REQUEST);
        res.success = true;
        res.message = "SET_MODE request accepted: " + requested_mode;
        record_vehicle_command_result(res.success, res.message);
        return true;
    }

    /* ── ARM / DISARM: 只接受请求并切入状态机，实际 arming 由 ARM_CTRL_REQUEST 发送 ── */
    if (command == "ARM" || command == "DISARM") {
        if (!runtime_flags.mavros_services_ready) {
            res.success = false;
            res.message = "MAVROS services unavailable";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }
        if (!vehicle_ready_for_command()) {
            res.success = false;
            res.message = "Vehicle command prerequisites not ready";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }
        if (vehicle_command_busy()) {
            res.success = false;
            res.message = "Vehicle command is busy";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }

        requested_arm_state = (command == "ARM");
        if (mavros_state.armed == requested_arm_state) {
            res.success = true;
            res.message = requested_arm_state ? "Vehicle already armed" : "Vehicle already disarmed";
            record_vehicle_command_result(res.success, res.message);
            return true;
        }

        transition_to(ARM_CTRL_REQUEST);
        res.success = true;
        res.message = requested_arm_state ? "ARM request accepted" : "DISARM request accepted";
        record_vehicle_command_result(res.success, res.message);
        return true;
    }

    res.success = false;
    res.message = "Unknown command: " + req.command;
    record_vehicle_command_result(res.success, res.message);
    return true;
}
