/**
 * @file safety_supervisor_node.cpp
 * @brief 安全监督节点：监控 MAVROS 连接、电池数据、定位有效性和速度管道状态。
 *
 * 按 NORMAL / WARNING / ERROR 三级发布安全状态，供 velocity_pipeline 和 velocity_manager 使用。
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <lite_quad_uav/SafetyStatus.h>
#include <lite_quad_uav/SafetySupervisorLogReport.h>
#include <lite_quad_uav/PipelineStatus.h>

/* SafetySupervisor ----------------------------------------------------------*/

/**
 * @brief 安全监督器：周期性评估安全规则并发布安全状态。
 */
class SafetySupervisor {
public:
    /**
     * @brief 构造函数，初始化参数、订阅话题和发布器。
     * @param[in] nh 节点句柄。
     * @param[in] pnh 私有节点句柄。
     */
    SafetySupervisor(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        position_timeout_ = pnh.param<double>("position_timeout", 5.0);
        battery_timeout_ = pnh.param<double>("battery_timeout", 5.0);
        startup_grace_period_ = pnh.param<double>("startup_grace_period", 15.0);
        double pub_rate = pnh.param<double>("publish_rate", 2.0);
        log_report_rate_ = pnh.param<double>("log_report_rate", 1.0);

        std::string mavros_ns = pnh.param<std::string>("mavros_ns", "/mavros");
        if (pub_rate <= 0.0) {
            ROS_WARN("[SafetySupervisor] publish_rate must be positive, reset to 2.0");
            pub_rate = 2.0;
        }
        if (log_report_rate_ <= 0.0) {
            ROS_WARN("[SafetySupervisor] log_report_rate must be positive, reset to 1.0");
            log_report_rate_ = 1.0;
        }

        // ── 话题与服务名称 ──────────────────────────────────────────────────
        const std::string velocity_manager_status_topic = "/uav/control/velocity_manager_status";  // 速度管理管道状态
        const std::string safety_status_topic = "/uav/safety/status";             // 安全监督状态 (velocity_manager 消费)
        const std::string log_report_topic = "/uav/log_reports/safety_supervisor"; // 节点日志报告 (logger_node 消费)

        // Subscribers
        state_sub_ = nh.subscribe(mavros_ns + "/state", 10,
            &SafetySupervisor::stateCallback, this);
        battery_sub_ = nh.subscribe(mavros_ns + "/battery", 10,
            &SafetySupervisor::batteryCallback, this);
        local_pos_sub_ = nh.subscribe(mavros_ns + "/local_position/pose", 10,
            &SafetySupervisor::localPosCallback, this);
        velocity_manager_status_sub_ = nh.subscribe(velocity_manager_status_topic, 10,
            &SafetySupervisor::velocityManagerStatusCallback, this);

        // Publisher
        status_pub_ = nh.advertise<lite_quad_uav::SafetyStatus>(safety_status_topic, 10);
        log_report_pub_ = nh.advertise<lite_quad_uav::SafetySupervisorLogReport>(log_report_topic, 10);

        // Timer
        timer_ = nh.createTimer(ros::Duration(1.0 / pub_rate),
            &SafetySupervisor::timerCallback, this);
        log_report_timer_ = nh.createTimer(ros::Duration(1.0 / log_report_rate_),
            &SafetySupervisor::logReportTimerCallback, this);

        start_time_ = ros::Time::now();

        ROS_INFO("[SafetySupervisor] Initialized, mavros_ns=%s, grace_period=%.1fs",
                 mavros_ns.c_str(), startup_grace_period_);
    }

private:
    double position_timeout_;                  /**< 定位数据超时阈值 (s) */
    double battery_timeout_;                    /**< 电池数据超时阈值 (s) */
    double startup_grace_period_;               /**< 启动宽限期 (s)，宽限期内 MAVROS 未连接和定位无效仅升级为 WARNING */
    double log_report_rate_;                    /**< 日志报告发布频率 (Hz) */
    ros::Time start_time_;                      /**< 节点启动时间 */

    mavros_msgs::State mavros_state_;           /**< 缓存的 MAVROS 飞控状态 */
    bool state_received_ = false;               /**< 是否已收到 MAVROS 状态 */
    sensor_msgs::BatteryState battery_;         /**< 缓存的电池数据 */
    bool battery_received_ = false;             /**< 是否已收到电池数据 */
    ros::Time last_battery_time_;               /**< 上次收到电池数据的时间 */
    geometry_msgs::PoseStamped local_pos_;      /**< 缓存的本地位置 */
    bool position_received_ = false;            /**< 是否已收到位置数据 */
    ros::Time last_position_time_;              /**< 上次收到位置数据的时间 */
    lite_quad_uav::PipelineStatus velocity_manager_status_;  /**< 速度管理管道状态缓存 */
    bool velocity_manager_status_received_ = false;          /**< 是否已收到管道状态 */
    lite_quad_uav::SafetyStatus last_safety_status_;          /**< 最近一次安全状态缓存 */
    bool safety_status_published_ = false;                    /**< 是否已发布过安全状态 */

    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber velocity_manager_status_sub_;
    // Publisher
    ros::Publisher status_pub_;
    ros::Publisher log_report_pub_;
    // Timer
    ros::Timer timer_;
    ros::Timer log_report_timer_;

    bool batteryFresh(const ros::Time& now) const {
        return battery_received_ && (now - last_battery_time_).toSec() <= battery_timeout_;
    }

    bool positionFresh(const ros::Time& now) const {
        return position_received_ && (now - last_position_time_).toSec() <= position_timeout_;
    }

    bool startupGraceActive(const ros::Time& now) const {
        return (now - start_time_).toSec() < startup_grace_period_;
    }

    /* Callbacks / 回调 ---------------------------------------------------------*/

    /**
     * @brief 缓存 MAVROS 飞控状态。
     * @param[in] msg MAVROS state 消息。
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        mavros_state_ = *msg;
        state_received_ = true;
    }

    /**
     * @brief 缓存电池数据并刷新时间戳。
     * @param[in] msg 电池状态消息。
     */
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
        battery_ = *msg;
        battery_received_ = true;
        last_battery_time_ = ros::Time::now();
    }

    /**
     * @brief 缓存本地位置并刷新时间戳。
     * @param[in] msg 本地位置消息。
     */
    void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        local_pos_ = *msg;
        position_received_ = true;
        last_position_time_ = ros::Time::now();
    }

    /**
     * @brief 缓存速度管理管道状态。
     * @param[in] msg 管道状态消息。
     */
    void velocityManagerStatusCallback(const lite_quad_uav::PipelineStatus::ConstPtr& msg) {
        velocity_manager_status_ = *msg;
        velocity_manager_status_received_ = true;
    }

    /* Timer / 定时检查 --------------------------------------------------------*/

    /**
     * @brief 周期性安全规则检查，按优先级从高到低评估并发布安全状态。
     *
     * @note 安全规则按顺序评估（MAVROS连接 → 电池 → 定位 → 管道看门狗），
     *       高优先级规则设置 level 后，低优先级规则仅在 level==NORMAL 时升级，
     *       因此同时发生的多个异常只会报告最高优先级的那一个。
     */
    void timerCallback(const ros::TimerEvent& /*event*/) {
        lite_quad_uav::SafetyStatus status;
        ros::Time now = ros::Time::now();
        status.header.stamp = now;
        status.level = "NORMAL";
        status.safe_to_fly = true;
        status.reason = "";

        bool in_grace = startupGraceActive(now);

        // Rule 1: MAVROS not connected
        if (!state_received_ || !mavros_state_.connected) {
            if (in_grace) {
                // During startup grace: WARNING, not ERROR
                status.level = "WARNING";
                status.safe_to_fly = true;
                status.reason = "MAVROS not connected (startup grace)";
            } else {
                status.level = "ERROR";
                status.safe_to_fly = false;
                status.reason = "MAVROS not connected";
            }
        }

        // Rule 2: Battery data unavailable -> WARNING
        if (!batteryFresh(now)) {
            if (status.level == "NORMAL") {
                status.level = "WARNING";
                status.reason = "Battery data unavailable";
            }
        }

        // Rule 3: Position unavailable
        if (!positionFresh(now)) {
            if (in_grace) {
                // During startup grace: WARNING, not ERROR
                if (status.level == "NORMAL") {
                    status.level = "WARNING";
                    status.reason = "Position unavailable (startup grace)";
                }
            } else {
                status.level = "ERROR";
                status.safe_to_fly = false;
                status.reason = "Position unavailable";
            }
        }

        // Rule 4: Velocity manager watchdog active -> WARNING
        if (velocity_manager_status_received_ && velocity_manager_status_.watchdog_active) {
            if (status.level == "NORMAL") {
                status.level = "WARNING";
                status.reason = "Velocity manager watchdog active";
            }
        }

        status_pub_.publish(status);
        last_safety_status_ = status;
        safety_status_published_ = true;
    }

    /**
     * @brief 周期发布安全监督日志报告，供 logger_node 集中展示。
     * @param[in] event 定时器事件。
     */
    void logReportTimerCallback(const ros::TimerEvent& /*event*/) {
        ros::Time now = ros::Time::now();

        lite_quad_uav::SafetySupervisorLogReport report;
        report.header.stamp = now;
        report.level = safety_status_published_ ? last_safety_status_.level : "UNKNOWN";
        report.safe_to_fly = safety_status_published_ && last_safety_status_.safe_to_fly;

        report.mavros_state_received = state_received_;
        report.mavros_connected = state_received_ && mavros_state_.connected;
        report.battery_fresh = batteryFresh(now);
        report.local_position_fresh = positionFresh(now);
        report.velocity_manager_status_received = velocity_manager_status_received_;
        report.velocity_watchdog_active = velocity_manager_status_received_ && velocity_manager_status_.watchdog_active;
        report.startup_grace_active = startupGraceActive(now);
        report.reason = safety_status_published_ ? last_safety_status_.reason : "No safety status published yet";

        log_report_pub_.publish(report);
    }
};

/* Main function / 主函数 -----------------------------------------------------*/

/**
 * @brief 安全监督节点入口。
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "safety_supervisor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    SafetySupervisor ss(nh, pnh);
    ros::spin();
    return 0;
}
