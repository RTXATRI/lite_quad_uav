/**
 * @file velocity_manager_node.cpp
 * @brief 底层速度管理节点：等待飞控可靠连接后，固定频率发布速度 setpoint 和管道状态。
 *
 * 本节点是常规 MAVROS 速度 setpoint 出口。未确认可靠连接前不向 MAVROS 发布 setpoint，
 * 避免在飞控链路未准备好时提前写控制话题。
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <lite_quad_uav/PipelineStatus.h>
#include <lite_quad_uav/SafetyStatus.h>
#include <lite_quad_uav/VehicleStatus.h>
#include <lite_quad_uav/VelocityCommand.h>
#include <lite_quad_uav/VelocityManagerLogReport.h>
#include <lite_quad_uav/velocity/velocity_pipeline.hpp>

/* Private typedef / 私有类型定义 --------------------------------------------*/
enum VelocityManagerState {
    WAIT_VEHICLE_STATUS,       // 等待 connect_manager_node 发布车辆状态
    WAIT_RELIABLE_CONNECTION,  // 等待 VehicleStatus.connected
    STREAM_SAFE_HOLD,          // 可靠连接后发布默认 0 速度 setpoint
    STREAM_ACTIVE,             // 发布候选速度命令经过管道处理后的 setpoint
    CONNECTION_LOST            // 已开始发布后连接失效，停止向 MAVROS 发布 setpoint
};

/* Private variables / 私有变量 ----------------------------------------------*/

static VelocityManagerState velocity_manager_state = WAIT_VEHICLE_STATUS;
static PipelineParams pipeline_params;
static VelocityPipeline pipeline;

static std::string mavros_ns = "/mavros";
static double vehicle_status_timeout = 2.0;
static double log_report_rate = 1.0;

static lite_quad_uav::VehicleStatus vehicle_status;
static bool vehicle_status_received = false;
static ros::Time last_vehicle_status_time;

static lite_quad_uav::VelocityCommand last_candidate_command;
static bool candidate_command_received = false;
static ros::Time last_candidate_command_time;
static lite_quad_uav::VelocityCommand last_output_command;
static bool output_command_received = false;
static lite_quad_uav::PipelineStatus last_pipeline_status;
static bool pipeline_status_received = false;

static ros::Subscriber candidate_sub;
static ros::Subscriber safety_sub;
static ros::Subscriber vehicle_status_sub;
static ros::Publisher mavros_vel_pub;
static ros::Publisher velocity_manager_status_pub;
static ros::Publisher log_report_pub;

/* Private function prototypes / 私有函数声明 --------------------------------*/

static const char* velocity_manager_state_to_string(VelocityManagerState state);
static void transition_to(VelocityManagerState next_state);
static bool vehicle_status_fresh(const ros::Time& now);
static bool reliable_connection_ready(const ros::Time& now);
static bool candidate_command_fresh(const ros::Time& now);
static void publish_waiting_velocity_manager_status(const ros::Time& now, const std::string& message);
static void publish_mavros_setpoint_and_status(const ros::Time& now);
static void candidate_cb(const lite_quad_uav::VelocityCommand::ConstPtr& msg);
static void safety_cb(const lite_quad_uav::SafetyStatus::ConstPtr& msg);
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg);
static void log_report_timer_cb(const ros::TimerEvent& event);

/* Main function / 主函数 -----------------------------------------------------*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 速度管道参数只影响 setpoint 输出频率、命令超时和限幅，不改变对外接口。
    pipeline_params.publish_rate = pnh.param<double>("publish_rate", 50.0);
    pipeline_params.command_timeout = pnh.param<double>("command_timeout", 0.5);
    pipeline_params.max_xy_speed = pnh.param<double>("max_xy_speed", 1.0);
    pipeline_params.max_z_up_speed = pnh.param<double>("max_z_up_speed", 0.6);
    pipeline_params.max_z_down_speed = pnh.param<double>("max_z_down_speed", 0.6);
    pipeline_params.max_yaw_rate = pnh.param<double>("max_yaw_rate", 1.0);
    mavros_ns = pnh.param<std::string>("mavros_ns", "/mavros");
    vehicle_status_timeout = pnh.param<double>("vehicle_status_timeout", 2.0);
    log_report_rate = pnh.param<double>("log_report_rate", 1.0);

    if (pipeline_params.publish_rate <= 0.0) {
        ROS_WARN("[Velocity_Manager_node] publish_rate must be positive, reset to 50.0");
        pipeline_params.publish_rate = 50.0;
    }
    if (pipeline_params.command_timeout <= 0.0) {
        ROS_WARN("[Velocity_Manager_node] command_timeout must be positive, reset to 0.5");
        pipeline_params.command_timeout = 0.5;
    }
    if (vehicle_status_timeout <= 0.0) {
        ROS_WARN("[Velocity_Manager_node] vehicle_status_timeout must be positive, reset to 2.0");
        vehicle_status_timeout = 2.0;
    }
    if (log_report_rate <= 0.0) {
        ROS_WARN("[Velocity_Manager_node] log_report_rate must be positive, reset to 1.0");
        log_report_rate = 1.0;
    }

    pipeline.setParams(pipeline_params);

    // ── 话题与服务名称 ──────────────────────────────────────────────────
    const std::string candidate_velocity_topic = "/uav/control/candidate_velocity";  // 候选速度命令 (controller 发布)
    const std::string safety_status_topic = "/uav/safety/status";             // 安全监督状态 (safety_supervisor 发布)
    const std::string vehicle_status_topic = "/uav/vehicle/status";           // 车辆连接状态 (connect_manager 发布)
    const std::string velocity_manager_status_topic = "/uav/control/velocity_manager_status"; // 速度管理管道状态
    const std::string log_report_topic = "/uav/log_reports/velocity_manager"; // 节点日志报告 (logger_node 消费)

    // 输入回调只缓存候选速度、安全状态和车辆连接状态，主循环统一决定是否发布 setpoint。
    candidate_sub = nh.subscribe(candidate_velocity_topic, 10, candidate_cb);
    safety_sub = nh.subscribe(safety_status_topic, 10, safety_cb);
    vehicle_status_sub = nh.subscribe(vehicle_status_topic, 10, vehicle_status_cb);

    mavros_vel_pub = nh.advertise<geometry_msgs::Twist>(mavros_ns + "/setpoint_velocity/cmd_vel_unstamped", 10);
    velocity_manager_status_pub = nh.advertise<lite_quad_uav::PipelineStatus>(velocity_manager_status_topic, 10);
    log_report_pub = nh.advertise<lite_quad_uav::VelocityManagerLogReport>(log_report_topic, 10);
    ros::Timer log_report_timer = nh.createTimer(ros::Duration(1.0 / log_report_rate), log_report_timer_cb);

    ROS_INFO("[Velocity_Manager_node] Initialized, rate = %.0f, command_timeout = %.2f, mavros_ns = %s", pipeline_params.publish_rate, pipeline_params.command_timeout, mavros_ns.c_str());

    ros::Rate rate(pipeline_params.publish_rate);
    while (ros::ok()) {
        ros::spinOnce();
        ros::Time now = ros::Time::now();

        // ── 主状态机 ──────────────────────────────────────────────────────────
        // 流程：等待 vehicle_status → 等待可靠连接 → 发布 setpoint (SAFE_HOLD/ACTIVE)
        // 连接丢失后停止向 MAVROS 发布，恢复后重新进入流式发布状态。
        switch (velocity_manager_state) {

        /* ── 阶段 1: 等待上游数据 ─────────────────────────────────────────── */

        case WAIT_VEHICLE_STATUS:
            // 分支 1: 尚未收到任何 vehicle_status → 持续等待
            if (!vehicle_status_received) {
                ROS_INFO_THROTTLE(5.0, "[Velocity_Manager_node] Waiting for vehicle status...");
                publish_waiting_velocity_manager_status(now, "Waiting for vehicle status");
            }
            // 分支 2: 已收到 status 但连接条件不满足 → 升级到 WAIT_RELIABLE_CONNECTION
            else if (!reliable_connection_ready(now)) {
                ROS_INFO("[Velocity_Manager_node] Vehicle status received, waiting for reliable connection");
                transition_to(WAIT_RELIABLE_CONNECTION);
                publish_waiting_velocity_manager_status(now, "Waiting for reliable vehicle connection");
            }
            // 分支 3: 可靠连接已建立 → 按候选命令是否新鲜直接进入 STREAM_ACTIVE 或 STREAM_SAFE_HOLD
            else {
                VelocityManagerState next_state = candidate_command_fresh(now) ? STREAM_ACTIVE : STREAM_SAFE_HOLD;
                ROS_INFO("[Velocity_Manager_node] Reliable connection established, publishing %s setpoint", next_state == STREAM_ACTIVE ? "active velocity" : "SAFE_HOLD");
                transition_to(next_state);
                publish_mavros_setpoint_and_status(now);
            }
            break;

        case WAIT_RELIABLE_CONNECTION:
            // 分支 1: vehicle_status 丢失 → 降级回 WAIT_VEHICLE_STATUS
            if (!vehicle_status_received) {
                ROS_WARN("[Velocity_Manager_node] Vehicle status unavailable, return to WAIT_VEHICLE_STATUS");
                transition_to(WAIT_VEHICLE_STATUS);
                ROS_INFO_THROTTLE(5.0, "[Velocity_Manager_node] Waiting for vehicle status...");
                publish_waiting_velocity_manager_status(now, "Waiting for vehicle status");
            }
            // 分支 2: 连接条件仍不满足 → 持续等待
            else if (!reliable_connection_ready(now)) {
                ROS_INFO_THROTTLE(5.0, "[Velocity_Manager_node] Waiting for reliable vehicle connection...");
                publish_waiting_velocity_manager_status(now, "Waiting for reliable vehicle connection");
            }
            // 分支 3: 可靠连接已建立 → 按候选命令新鲜度进入流式发布
            else {
                VelocityManagerState next_state = candidate_command_fresh(now) ? STREAM_ACTIVE : STREAM_SAFE_HOLD;
                ROS_INFO("[Velocity_Manager_node] Reliable connection established, publishing %s setpoint", next_state == STREAM_ACTIVE ? "active velocity" : "SAFE_HOLD");
                transition_to(next_state);
                publish_mavros_setpoint_and_status(now);
            }
            break;

        /* ── 阶段 2: 流式发布 setpoint ────────────────────────────────────── */

        case STREAM_SAFE_HOLD:
            // 分支 1: 连接丢失 → 停止发布，进入 CONNECTION_LOST
            if (!reliable_connection_ready(now)) {
                ROS_WARN("[Velocity_Manager_node] Reliable connection lost, stop publishing MAVROS setpoints");
                transition_to(CONNECTION_LOST);
                publish_waiting_velocity_manager_status(now, "Reliable vehicle connection lost");
            }
            // 分支 2: 候选命令到达 → 升级到 STREAM_ACTIVE 发布实际速度
            else if (candidate_command_fresh(now)) {
                ROS_INFO("[Velocity_Manager_node] Candidate command active, publishing active velocity setpoint");
                transition_to(STREAM_ACTIVE);
                publish_mavros_setpoint_and_status(now);
            }
            // 分支 3: 稳态：发布管道处理后的安全保持速度（pipeline watchdog 输出零速度）
            else {
                publish_mavros_setpoint_and_status(now);
            }
            break;

        case STREAM_ACTIVE:
            // 分支 1: 连接丢失 → 停止发布，进入 CONNECTION_LOST
            if (!reliable_connection_ready(now)) {
                ROS_WARN("[Velocity_Manager_node] Reliable connection lost, stop publishing MAVROS setpoints");
                transition_to(CONNECTION_LOST);
                publish_waiting_velocity_manager_status(now, "Reliable vehicle connection lost");
            }
            // 分支 2: 候选命令过期/无效 → 降级到 STREAM_SAFE_HOLD 发布安全保持速度
            else if (!candidate_command_fresh(now)) {
                ROS_INFO("[Velocity_Manager_node] Candidate command inactive, publishing SAFE_HOLD setpoint");
                transition_to(STREAM_SAFE_HOLD);
                publish_mavros_setpoint_and_status(now);
            }
            // 分支 3: 稳态：发布管道处理后的候选速度
            else {
                publish_mavros_setpoint_and_status(now);
            }
            break;

        /* ── 异常恢复 ─────────────────────────────────────────────────────── */

        case CONNECTION_LOST:
            // 分支 1: vehicle_status 完全丢失 → 降级回 WAIT_VEHICLE_STATUS
            if (!vehicle_status_received) {
                ROS_WARN("[Velocity_Manager_node] Vehicle status unavailable, return to WAIT_VEHICLE_STATUS");
                transition_to(WAIT_VEHICLE_STATUS);
                ROS_INFO_THROTTLE(5.0, "[Velocity_Manager_node] Waiting for vehicle status...");
                publish_waiting_velocity_manager_status(now, "Waiting for vehicle status");
            }
            // 分支 2: 连接尚未恢复 → 持续等待
            else if (!reliable_connection_ready(now)) {
                ROS_INFO_THROTTLE(5.0, "[Velocity_Manager_node] Waiting for reliable connection recovery...");
                publish_waiting_velocity_manager_status(now, "Reliable vehicle connection lost");
            }
            // 分支 3: 连接恢复 → 按候选命令新鲜度重新进入流式发布
            else {
                VelocityManagerState next_state = candidate_command_fresh(now) ? STREAM_ACTIVE : STREAM_SAFE_HOLD;
                ROS_INFO("[Velocity_Manager_node] Reliable connection recovered, publishing %s setpoint", next_state == STREAM_ACTIVE ? "active velocity" : "SAFE_HOLD");
                transition_to(next_state);
                publish_mavros_setpoint_and_status(now);
            }
            break;
        }

        rate.sleep();
    }

    return 0;
}

/* Private function implementations / 私有函数实现 ----------------------------*/

/**
 * @brief 将速度管理状态枚举转为字符串。
 * @param[in] state 状态枚举值。
 * @return 状态名称字符串。
 */
static const char* velocity_manager_state_to_string(VelocityManagerState state) {
    switch (state) {
    case WAIT_VEHICLE_STATUS:      return "WAIT_VEHICLE_STATUS";
    case WAIT_RELIABLE_CONNECTION: return "WAIT_RELIABLE_CONNECTION";
    case STREAM_SAFE_HOLD:         return "STREAM_SAFE_HOLD";
    case STREAM_ACTIVE:            return "STREAM_ACTIVE";
    case CONNECTION_LOST:          return "CONNECTION_LOST";
    }
    return "UNKNOWN";
}

/**
 * @brief 切换到下一状态，记录进入时间。
 * @param[in] next_state 目标状态。
 *
 * @note 同状态不切换。velocity_manager 只负责持续发布 setpoint，不请求飞控模式。
 */
static void transition_to(VelocityManagerState next_state) {
    if (velocity_manager_state == next_state) {
        return;
    }

    velocity_manager_state = next_state;
}

/**
 * @brief 检查 vehicle_status 是否新鲜（已收到且未超时）。
 * @param[in] now 当前时间。
 * @return true=vehicle_status 在超时窗口内。
 */
static bool vehicle_status_fresh(const ros::Time& now) {
    return vehicle_status_received && (now - last_vehicle_status_time).toSec() <= vehicle_status_timeout;
}

/**
 * @brief 判断可靠连接是否就绪：vehicle_status 新鲜 + connected。
 * @param[in] now 当前时间。
 * @return true=可靠连接。
 *
 * @note 不检查 vehicle_status.ready，否则与 connect_manager_node 的 STREAM_WAIT_SETPOINT_STREAM
 *       状态形成循环等待：connect_manager 等 velocity_manager 发布 setpoint，
 *       velocity_manager 等 connect_manager 发布 ready=true。
 */
static bool reliable_connection_ready(const ros::Time& now) {
    return vehicle_status_fresh(now) && vehicle_status.connected;
}

/**
 * @brief 检查候选速度命令是否新鲜：已收到 + valid=true + 未超时。
 * @param[in] now 当前时间。
 * @return true=候选命令可被管道消费。
 */
static bool candidate_command_fresh(const ros::Time& now) {
    return candidate_command_received && last_candidate_command.valid && (now - last_candidate_command_time).toSec() <= pipeline_params.command_timeout;
}

/**
 * @brief 在等待状态下发布管道状态（watchdog_active=true）。
 * @param[in] now 当前时间。
 * @param[in] message 状态描述。
 *
 * watchdog_active=true 告知 connect_manager_node 本节点尚未进入流式发布，
 * connect_manager 的 STREAM_WAIT_SETPOINT_STREAM 状态会据此等待。
 */
static void publish_waiting_velocity_manager_status(const ros::Time& now, const std::string& message) {
    lite_quad_uav::PipelineStatus status;
    status.header.stamp = now;
    status.state = velocity_manager_state_to_string(velocity_manager_state);
    status.command_source = "";
    status.watchdog_active = true;
    status.last_command_age_sec = candidate_command_received ? (now - last_candidate_command_time).toSec() : 0.0;
    status.message = message;
    velocity_manager_status_pub.publish(status);
    last_pipeline_status = status;
    pipeline_status_received = true;
}

/**
 * @brief 核心发布函数：执行管道处理链后同时发布 MAVROS setpoint 和管道状态。
 * @param[in] now 当前时间。
 *
 * 处理流程：pipeline.process(now) 执行 watchdog → safety_filter → limits，
 * 输出最终速度命令，转换为 geometry_msgs/Twist 发布到 MAVROS。
 */
static void publish_mavros_setpoint_and_status(const ros::Time& now) {
    lite_quad_uav::VelocityCommand output = pipeline.process(now);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = output.vx;
    cmd_vel.linear.y = output.vy;
    cmd_vel.linear.z = output.vz;
    cmd_vel.angular.z = output.yaw_rate;
    mavros_vel_pub.publish(cmd_vel);

    lite_quad_uav::PipelineStatus status = pipeline.getStatus();
    status.header.stamp = now;
    velocity_manager_status_pub.publish(status);

    last_output_command = output;
    output_command_received = true;
    last_pipeline_status = status;
    pipeline_status_received = true;
}

/**
 * @brief 周期发布速度管理节点日志报告，供 logger_node 集中展示。
 * @param[in] event 定时器事件。
 */
static void log_report_timer_cb(const ros::TimerEvent& /*event*/) {
    ros::Time now = ros::Time::now();

    lite_quad_uav::VelocityManagerLogReport report;
    report.header.stamp = now;
    report.state = velocity_manager_state_to_string(velocity_manager_state);
    report.vehicle_status_fresh = vehicle_status_fresh(now);
    report.reliable_connection = reliable_connection_ready(now);
    report.candidate_fresh = candidate_command_fresh(now);

    report.watchdog_active = pipeline_status_received ? last_pipeline_status.watchdog_active : true;
    report.command_source = pipeline_status_received ? last_pipeline_status.command_source : "";
    report.last_command_age_sec = candidate_command_received ? (now - last_candidate_command_time).toSec() : 0.0;

    report.output_vx = output_command_received ? last_output_command.vx : 0.0;
    report.output_vy = output_command_received ? last_output_command.vy : 0.0;
    report.output_vz = output_command_received ? last_output_command.vz : 0.0;
    report.output_yaw_rate = output_command_received ? last_output_command.yaw_rate : 0.0;
    report.output_valid = output_command_received && last_output_command.valid;
    report.message = pipeline_status_received ? last_pipeline_status.message : "No velocity manager status published yet";

    log_report_pub.publish(report);
}

/**
 * @brief 候选速度命令回调：同时缓存到 last_candidate_command 和 pipeline。
 * @param[in] msg 候选速度命令。
 *
 * 双写原因：last_candidate_command 供 candidate_command_fresh() 判断新鲜度，
 * pipeline.updateCommand() 供 pipeline.process() 消费。
 */
static void candidate_cb(const lite_quad_uav::VelocityCommand::ConstPtr& msg) {
    last_candidate_command = *msg;
    last_candidate_command_time = ros::Time::now();
    candidate_command_received = true;
    pipeline.updateCommand(*msg);
}

/**
 * @brief 安全状态回调：更新管道的安全过滤输入。
 * @param[in] msg 安全监督状态。
 */
static void safety_cb(const lite_quad_uav::SafetyStatus::ConstPtr& msg) {
    pipeline.updateSafetyStatus(*msg);
}

/**
 * @brief 车辆连接状态回调：缓存 vehicle_status 供状态机判断连接条件。
 * @param[in] msg 车辆状态消息。
 */
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg) {
    vehicle_status = *msg;
    last_vehicle_status_time = ros::Time::now();
    vehicle_status_received = true;
}
