/**
 * @file target_estimator_node.cpp
 * @brief 通用 AprilTag 识别结果整理节点：将 tag 位姿从相机坐标系转换到机体系/水平体系，
 *        按 tag_id 维护 alpha-beta 滤波跟踪状态和置信度评估。
 *
 * 本节点订阅 apriltag_ros 的检测结果，将 tag 位姿从相机坐标系转换到机体系，
 * 再经 IMU 姿态补偿到水平体系 (去 roll/pitch 保留 yaw)，并按 tag_id 维护
 * alpha-beta 滤波跟踪状态和综合置信度。节点只做 sensing 层信息整理，不发布控制指令。
 */

/* Includes / 头文件 ---------------------------------------------------------*/
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <lite_quad_uav/AprilTagTarget.h>
#include <lite_quad_uav/AprilTagTargetArray.h>
#include <lite_quad_uav/TargetEstimatorLogReport.h>
#include <lite_quad_uav/VehicleStatus.h>

/* Private typedef / 私有类型定义 --------------------------------------------*/

enum class TargetEstimatorNodeState {
    INITIALIZING = 0,  // 初始化阶段：等待可靠车辆连接
    NORMAL,            // 正常运行：处理 tag 检测并更新跟踪
    REINITIALIZING     // 重初始化：车辆连接丢失后等待恢复
};

struct TargetEstimatorRuntimeFlags {
    bool vehicle_status_fresh = false;        /**< vehicle_status 话题数据新鲜 (未超时) */
    bool vehicle_connected = false;           /**< 飞控已连接 (vehicle_status.connected) */
    bool reliable_vehicle_connection = false; /**< 可靠车辆连接 (fresh + connected) */
    bool tag_detection_received = false;      /**< 是否曾收到过 tag 检测数据 */
    bool tag_detection_fresh = false;         /**< tag 检测数据新鲜 (未超时) */
};

struct TagTrack {
    int tag_id = -1;                              /**< Tag 编号，对应 AprilTag family 中的 ID */
    std::string state = "LOST";                   /**< 跟踪状态: LOST / DETECTED / TRACKING / PREDICTING / REJECTED */
    bool detected = false;                        /**< 当前周期是否检测到该 tag */
    bool pose_valid = false;                      /**< 当前位姿是否有效 (confidence >= threshold) */
    bool has_pose = false;                        /**< 是否曾获得过位姿 (至少一次成功转换) */
    geometry_msgs::Pose pose_body_raw;            /**< 机体系原始位姿 (未经滤波) */
    geometry_msgs::Point position_level_raw;      /**< 水平体系原始位置 (经姿态补偿，未经滤波) */
    geometry_msgs::Point position_level_filtered; /**< 水平体系滤波后位置 (alpha-beta 输出) */
    geometry_msgs::Vector3 velocity_level_filtered;/**< 水平体系滤波后速度 (alpha-beta 输出，m/s) */
    bool filter_initialized = false;              /**< alpha-beta 滤波器是否已初始化 */
    double confidence = 0.0;                      /**< 综合置信度 [0,1]，低于 threshold 时 pose_valid=false */
    double residual_norm = 0.0;                   /**< 最近一次测量残差范数 (m)，用于跳变检测和置信度计算 */
    double detection_age = 0.0;                   /**< 检测时间戳相对当前时间的延迟 (s) */
    double imu_age = -1.0;                        /**< IMU 数据相对当前时间的延迟 (s)，-1 表示未收到 */
    uint32_t stable_count = 0;                    /**< 连续稳定检测帧计数，达到 stable_frame_threshold 升级为 TRACKING */
    uint32_t lost_count = 0;                      /**< 连续丢失帧计数，超过 lost_frame_threshold 进入 LOST */
    ros::Time last_seen_time;                     /**< 最近一次检测到该 tag 的时间戳 */
    ros::Time last_seen_recv_time;                /**< 最近一次收到该 tag 检测的本节点接收时间 */
    ros::Time last_update_time;                   /**< 最近一次更新该 track 的时间 (含 PREDICTING) */
    ros::Time last_filter_time;                   /**< 最近一次滤波更新的时间戳 (alpha-beta dt 基准) */
    ros::Time last_filter_recv_time;              /**< 最近一次滤波更新的本节点接收时间 */
    ros::Time last_processed_detection_stamp;     /**< 最近一次已处理的检测消息时间戳 (去重) */
    uint64_t last_processed_detection_sequence = 0;/**< 最近一次已处理的检测数组序号 (接收时间域去重) */
};

/* Private variables / 私有变量 ----------------------------------------------*/

static TargetEstimatorNodeState node_state = TargetEstimatorNodeState::INITIALIZING; /**< 节点运行状态 */
static TargetEstimatorRuntimeFlags runtime_flags;                                    /**< 运行时健康标志 */

static std::string tag_detection_topic = "/tag_detections";     /**< AprilTag 检测结果话题名 */
static std::string vehicle_status_topic = "/uav/vehicle/status";/**< 车辆连接状态话题名 */
static std::string imu_topic = "/mavros/imu/data";              /**< IMU 姿态数据话题名 */
static std::string camera_frame = "camera_optical_frame";       /**< 相机光学坐标系 TF frame ID */
static std::string body_frame = "base_link";                    /**< 机体系 TF frame ID */
static std::string level_frame = "level_body";                  /**< 水平机体系 TF frame ID (去 roll/pitch) */

static bool use_tf_extrinsic = false;          /**< true=从 TF 树查外参; false=使用手动外参参数 */
static double camera_translation_x = 0.0;      /**< 相机到机体系平移 X 分量 (m) */
static double camera_translation_y = 0.0;      /**< 相机到机体系平移 Y 分量 (m) */
static double camera_translation_z = 0.0;      /**< 相机到机体系平移 Z 分量 (m) */
static double camera_roll = 0.0;               /**< 相机到机体系旋转 roll (rad) */
static double camera_pitch = 0.0;              /**< 相机到机体系旋转 pitch (rad) */
static double camera_yaw = 0.0;                /**< 相机到机体系旋转 yaw (rad) */

static double runtime_check_rate = 20.0;       /**< 运行时健康检查频率 (Hz) */
static double estimator_update_rate = 30.0;    /**< 估计器主更新循环频率 (Hz) */
static double log_report_rate = 1.0;           /**< 日志报告发布频率 (Hz) */
static double vehicle_status_timeout = 2.0;    /**< vehicle_status 数据超时阈值 (s) */
static double tag_detection_timeout = 0.5;     /**< tag 检测数据超时阈值 (s) */
static double imu_timeout = 0.1;               /**< IMU 数据超时阈值 (s) */
static double min_valid_distance = 0.15;       /**< tag 有效距离下限 (m)，过近视为无效 */
static double max_valid_distance = 10.0;       /**< tag 有效距离上限 (m)，过远视为无效 */
static double max_position_jump = 1.0;         /**< 单次测量最大允许跳变 (m)，超限则拒绝该测量 */
static double alpha = 0.45;                    /**< alpha-beta 滤波位置增益 [0,1]，越大跟踪越灵敏但噪声越大 */
static double beta = 0.08;                     /**< alpha-beta 滤波速度增益 [0,+inf)，越大速度响应越快但越不平滑 */
static double min_filter_dt = 0.005;           /**< 滤波最小有效时间步长 (s)，过短则跳过本帧更新 */
static double max_filter_dt = 0.20;            /**< 滤波最大有效时间步长 (s)，超限则截断防止发散 */
static int stable_frame_threshold = 3;         /**< 升级为 TRACKING 状态所需的连续稳定帧数 */
static int lost_frame_threshold = 5;           /**< 超过后 track 进入 LOST 的连续丢失帧数 */
static double max_predict_time = 0.30;         /**< 丢失后允许预测的最大时长 (s)，超限进入 LOST */
static double track_retention_time = 1.0;      /**< LOST 后 track 保留在 map 中的时长 (s)，超限则删除 */
static double confidence_threshold = 0.5;      /**< 置信度阈值 [0,1]，低于此值 pose_valid=false */
static double tau_lost = 0.2;                  /**< 丢失后置信度指数衰减时间常数 (s)，越小衰减越快 */
static double tau_detection = 0.15;            /**< 检测延迟置信度指数衰减时间常数 (s)，越小对延迟越敏感 */
static double residual_scale = 0.5;            /**< 残差置信度指数衰减缩放因子 (m)，越小对残差越敏感 */
static double imu_stale_confidence_scale = 0.35;/**< IMU 超时时置信度乘数 [0,1]，补偿姿态补偿不确定度 */

static lite_quad_uav::VehicleStatus vehicle_status; /**< 缓存的车辆状态消息 */
static bool vehicle_status_received = false;        /**< 是否曾收到过 vehicle_status */
static ros::Time last_vehicle_status_time;          /**< 最近一次收到 vehicle_status 的时间戳 */
static ros::Time last_vehicle_status_recv_time;     /**< 最近一次收到 vehicle_status 的本节点接收时间 */

static sensor_msgs::Imu latest_imu;           /**< 缓存的最新 IMU 消息 */
static bool imu_received = false;             /**< 是否曾收到过 IMU 数据 */
static ros::Time last_imu_time;               /**< 最近一次收到 IMU 数据的时间戳 */
static ros::Time last_imu_recv_time;          /**< 最近一次收到 IMU 数据的本节点接收时间 */

static apriltag_ros::AprilTagDetectionArray latest_detections; /**< 缓存的最新 tag 检测数据 */
static bool tag_detection_received = false;   /**< 是否曾收到过 tag 检测数据 */
static ros::Time last_tag_detection_time;     /**< 最近一次收到 tag 检测数据的时间戳 */
static ros::Time last_tag_detection_recv_time;/**< 最近一次收到 tag 检测数据的本节点接收时间 */
static uint64_t latest_detection_sequence = 0;/**< tag 检测数组接收序号，用于定时器重复处理去重 */
static std::string latest_source_frame;       /**< 最新检测消息的 source frame */

static std::map<int, TagTrack> tag_tracks;    /**< 按 tag_id 索引的跟踪记录表 */
static std::vector<int> latest_detected_tag_ids; /**< 当前周期检测到的 tag ID 列表 */
static uint32_t last_detected_count = 0;      /**< 最近一次检测到的 tag 数量 */
static uint32_t last_tracked_count = 0;       /**< 最近一次处于跟踪状态的 tag 数量 */
static uint32_t last_valid_count = 0;         /**< 最近一次位姿有效的 tag 数量 */
static std::string last_message = "target estimator initializing"; /**< 最近一次状态描述消息 */

static std::unique_ptr<tf2_ros::Buffer> tf_buffer;        /**< TF 缓冲区 (仅 use_tf_extrinsic=true 时使用) */
static std::unique_ptr<tf2_ros::TransformListener> tf_listener; /**< TF 监听器 (仅 use_tf_extrinsic=true 时使用) */

static ros::Subscriber tag_detection_sub;     /**< AprilTag 检测结果订阅者 */
static ros::Subscriber vehicle_status_sub;    /**< 车辆状态订阅者 */
static ros::Subscriber imu_sub;               /**< IMU 数据订阅者 */
static ros::Publisher target_pub;             /**< 滤波后 tag 目标数组发布者 */
static ros::Publisher log_report_pub;         /**< 节点日志报告发布者 */

/* Private function prototypes / 私有函数声明 --------------------------------*/

static const char* node_state_to_string(TargetEstimatorNodeState state);
static void transition_to(TargetEstimatorNodeState next_state);
static void load_parameters(ros::NodeHandle& pnh);
static void validate_parameters();
static void print_startup_info();
static void tag_detection_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg);
static void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
static void update_runtime_check_flags(const ros::Time& now);
static void update_node_state(const ros::Time& now);
static void refresh_estimator_context(const ros::Time& now);
static void process_latest_detections(const ros::Time& now);
static void update_missing_tracks(const std::set<int>& detected_ids, const ros::Time& now);
static void mark_all_tracks_invalid(const std::string& message, const ros::Time& now);
static bool get_body_to_camera_transform(const ros::Time& stamp, tf2::Transform* tf_body_camera);
static tf2::Transform pose_msg_to_tf(const geometry_msgs::Pose& pose);
static geometry_msgs::Pose tf_to_pose_msg(const tf2::Transform& transform);
static geometry_msgs::Point vector_to_point(const tf2::Vector3& vector);
static tf2::Vector3 point_to_vector(const geometry_msgs::Point& point);
static geometry_msgs::Point add_velocity_dt(const geometry_msgs::Point& point, const geometry_msgs::Vector3& velocity, double dt);
static geometry_msgs::Vector3 zero_vector3();
static bool imu_attitude_available(const ros::Time& now);
static double current_imu_age(const ros::Time& now);
static geometry_msgs::Point compensate_body_to_level(const geometry_msgs::Point& body_position, bool use_current_imu_attitude);
static bool update_track_filter(TagTrack& track, const geometry_msgs::Point& measurement, const ros::Time& detection_stamp, const ros::Time& filter_time);
static double compute_detection_confidence(const TagTrack& track, bool accepted, bool distance_valid, bool use_current_imu_attitude);
static double point_norm(const geometry_msgs::Point& point);
static double clamp(double value, double min_value, double max_value);
static double age_sec(const ros::Time& now, const ros::Time& stamp);
static double age_sec_or_fallback(const ros::Time& now, const ros::Time& stamp, double fallback_age, const char* label);
static bool safe_delta_sec(const ros::Time& end, const ros::Time& start, double* dt, const char* label);
static double stable_confidence(uint32_t stable_count);
static double lost_confidence(double lost_time);
static void publish_target_array(const ros::Time& now);
static lite_quad_uav::AprilTagTarget make_target_msg(const TagTrack& track);
static void update_output_counts();
static void runtime_check_timer_cb(const ros::TimerEvent& event);
static void estimator_update_timer_cb(const ros::TimerEvent& event);
static void log_report_timer_cb(const ros::TimerEvent& event);

/* Main function / 主函数 -----------------------------------------------------*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_estimator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    load_parameters(pnh);
    validate_parameters();

    // use_tf_extrinsic=true 时需要 TF 缓冲区和监听器查外参变换。
    if (use_tf_extrinsic) {
        tf_buffer.reset(new tf2_ros::Buffer());
        tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));
    }

    // 输入回调只缓存 tag 检测、车辆状态和 IMU 姿态，估计器主循环统一消费。
    tag_detection_sub = nh.subscribe(tag_detection_topic, 10, tag_detection_cb);
    vehicle_status_sub = nh.subscribe(vehicle_status_topic, 10, vehicle_status_cb);
    imu_sub = nh.subscribe(imu_topic, 50, imu_cb);

    // 输出话题：滤波后的 tag 目标数组和节点日志报告。
    const std::string apriltag_target_topic = "/uav/sensing/apriltag_target";   // 滤波后 tag 目标数组 (controller 消费)
    const std::string log_report_topic = "/uav/log_reports/target_estimator";    // 节点日志报告 (logger_node 消费)
    target_pub = nh.advertise<lite_quad_uav::AprilTagTargetArray>(apriltag_target_topic, 10);
    log_report_pub = nh.advertise<lite_quad_uav::TargetEstimatorLogReport>(log_report_topic, 10);

    // 定时器：健康检查、估计器主更新、日志报告，避免回调中混入定时逻辑。
    ros::Timer runtime_check_timer = nh.createTimer(ros::Duration(1.0 / runtime_check_rate), runtime_check_timer_cb);
    ros::Timer estimator_update_timer = nh.createTimer(ros::Duration(1.0 / estimator_update_rate), estimator_update_timer_cb);
    ros::Timer log_report_timer = nh.createTimer(ros::Duration(1.0 / log_report_rate), log_report_timer_cb);

    print_startup_info();
    ros::spin();
    return 0;
}

/* Private function implementations / 私有函数实现 ---------------------------*/

/**
 * @brief 将节点状态枚举转为字符串。
 * @param[in] state 节点状态枚举值。
 * @return 状态名称字符串。
 */
static const char* node_state_to_string(TargetEstimatorNodeState state) {
    switch (state) {
        case TargetEstimatorNodeState::INITIALIZING: return "INITIALIZING";
        case TargetEstimatorNodeState::NORMAL: return "NORMAL";
        case TargetEstimatorNodeState::REINITIALIZING: return "REINITIALIZING";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 切换节点运行状态，同状态不切换。
 * @param[in] next_state 目标状态。
 *
 * @note 切换时输出 INFO 日志，用于调试状态转换时序。
 */
static void transition_to(TargetEstimatorNodeState next_state) {
    if (node_state == next_state) {
        return;
    }

    ROS_INFO("[TargetEstimator] node_state: %s -> %s",
             node_state_to_string(node_state), node_state_to_string(next_state));
    node_state = next_state;
}

/**
 * @brief 从 ROS 参数服务器加载所有运行参数。
 * @param[in] pnh 私有节点句柄 (~ 命名空间)。
 *
 * @note 参数分为五组：话题名、相机外参、时序/频率、滤波/跟踪、置信度。
 *       每个参数均有默认值，可在 launch 文件中覆盖。
 *       加载后由 validate_parameters() 统一校验范围。
 */
static void load_parameters(ros::NodeHandle& pnh) {
    // ── 话题名参数 ──
    // 这些话题名可配置，用于适配不同 launch 配置下的 topic 重映射。
    pnh.param<std::string>("tag_detection_topic", tag_detection_topic, "/tag_detections");
    // AprilTag 检测结果输入话题 (apriltag_ros 发布)，默认 /tag_detections
    pnh.param<std::string>("vehicle_status_topic", vehicle_status_topic, "/uav/vehicle/status");
    // connect_manager 发布的车辆连接状态，用于判断可靠连接后再处理检测
    pnh.param<std::string>("imu_topic", imu_topic, "/mavros/imu/data");
    // MAVROS IMU 姿态话题，用于将机体系位姿补偿到水平体系
    pnh.param<std::string>("camera_frame", camera_frame, "camera_optical_frame");
    // 相机光学坐标系 frame ID，检测消息 frame_id 和 TF 查询使用
    pnh.param<std::string>("body_frame", body_frame, "base_link");
    // 机体系 frame ID，TF 查询目标 frame 和输出消息 body_frame 字段
    pnh.param<std::string>("level_frame", level_frame, "level_body");
    // 水平机体系 frame ID (去除 roll/pitch 保留 yaw)，输出消息 level_frame 字段

    // ── 相机外参参数 ──
    // use_tf_extrinsic 选择外参来源：true 从 TF 树实时查询 (需有 camera->body 的 TF)，
    // false 使用以下手动外参参数。手动模式适用于无 TF 发布的简化部署场景。
    pnh.param<bool>("use_tf_extrinsic", use_tf_extrinsic, false);
    // 外参模式开关：true=TF 查询, false=手动参数。切换需重启节点。
    pnh.param<double>("camera_translation_x", camera_translation_x, 0.0);
    // 相机到机体系平移 X 分量 (m)，正值=相机在机体系 +X 方向。仅在 use_tf_extrinsic=false 时生效。
    pnh.param<double>("camera_translation_y", camera_translation_y, 0.0);
    // 相机到机体系平移 Y 分量 (m)，正值=相机在机体系 +Y 方向。仅在 use_tf_extrinsic=false 时生效。
    pnh.param<double>("camera_translation_z", camera_translation_z, 0.0);
    // 相机到机体系平移 Z 分量 (m)，正值=相机在机体系 +Z 方向。仅在 use_tf_extrinsic=false 时生效。
    pnh.param<double>("camera_roll", camera_roll, 0.0);
    // 相机到机体系旋转 roll (rad)，按 RPY 约定。仅在 use_tf_extrinsic=false 时生效。
    pnh.param<double>("camera_pitch", camera_pitch, 0.0);
    // 相机到机体系旋转 pitch (rad)。俯仰补偿偏差会导致水平位置系统性偏移。
    pnh.param<double>("camera_yaw", camera_yaw, 0.0);
    // 相机到机体系旋转 yaw (rad)。偏航偏差会导致水平体系下位置旋转误差。

    // ── 时序/频率参数 ──
    pnh.param<double>("runtime_check_rate", runtime_check_rate, 20.0);
    // 运行时健康检查定时器频率 (Hz)。影响 runtime_flags 刷新粒度，不影响估计器输出。
    pnh.param<double>("estimator_update_rate", estimator_update_rate, 30.0);
    // 估计器主更新循环频率 (Hz)。越高跟踪延迟越低，但 CPU 开销增大。
    // 应高于 tag 检测帧率以保证每帧检测都被处理。
    pnh.param<double>("log_report_rate", log_report_rate, 1.0);
    // 日志报告发布频率 (Hz)。仅供 logger_node 展示，不影响估计器逻辑。
    pnh.param<double>("vehicle_status_timeout", vehicle_status_timeout, 2.0);
    // vehicle_status 数据超时阈值 (s)。超时后 node_state 降级到 REINITIALIZING，
    // 所有 track 标记为 LOST。需大于 connect_manager 的 status_publish_rate 对应周期。
    pnh.param<double>("tag_detection_timeout", tag_detection_timeout, 0.5);
    // tag 检测数据超时阈值 (s)。超时后不再处理最新检测，仅更新丢失 track。
    // 应大于检测帧间隔 (如 30Hz 检测则帧间隔约 0.033s)，但小于需要判定的丢失时间。
    pnh.param<double>("imu_timeout", imu_timeout, 0.1);
    // IMU 数据超时阈值 (s)。超时后补偿使用水平假设 (不补偿 roll/pitch)，
    // 置信度乘以 imu_stale_confidence_scale 降低。MAVROS 默认 IMU 约 50-100Hz。

    // ── 滤波/跟踪参数 ──
    pnh.param<double>("min_valid_distance", min_valid_distance, 0.15);
    // tag 有效距离下限 (m)。距离低于此值视为无效检测 (可能为标定误差或近距离噪点)。
    // 与 max_valid_distance 配合构成有效距离窗口。
    pnh.param<double>("max_valid_distance", max_valid_distance, 10.0);
    // tag 有效距离上限 (m)。距离超过此值的检测精度不足以信赖。
    // 距离越远置信度越低 (c_dist 线性递减到 0.2)。
    pnh.param<double>("max_position_jump", max_position_jump, 1.0);
    // 单次测量最大允许跳变距离 (m)。测量残差超过此值时拒绝该帧 (state=REJECTED)，
    // 防止 AprilTag ID 冲突或检测跳变导致滤波器发散。
    pnh.param<double>("alpha", alpha, 0.45);
    // alpha-beta 滤波位置增益 [0,1]。越大跟踪响应越灵敏但噪声抑制越弱。
    // 典型值 0.3~0.5；与 beta 联合调参，alpha 过大会导致位置抖动。
    pnh.param<double>("beta", beta, 0.08);
    // alpha-beta 滤波速度增益 [0,+inf)。越大速度估计响应越快但越不平滑。
    // 典型值 0.05~0.15；beta 过大会导致速度估计震荡，影响 PREDICTING 精度。
    pnh.param<double>("min_filter_dt", min_filter_dt, 0.005);
    // 滤波最小有效时间步长 (s)。dt 小于此值时跳过滤波更新 (返回 true)，
    // 防止连续同帧检测导致 dt 接近 0 时速度发散。
    pnh.param<double>("max_filter_dt", max_filter_dt, 0.20);
    // 滤波最大有效时间步长 (s)。dt 超过此值时截断为 max_filter_dt，
    // 防止长时间未更新后单次 dt 过大导致预测跳跃。需大于 1/estimator_update_rate。
    pnh.param<int>("stable_frame_threshold", stable_frame_threshold, 3);
    // 连续稳定检测帧计数阈值。stable_count 达到此值后 track 从 DETECTED 升级为 TRACKING。
    // 增大可减少瞬时误检影响，但增加跟踪建立延迟。
    pnh.param<int>("lost_frame_threshold", lost_frame_threshold, 5);
    // 连续丢失帧计数阈值。lost_count 超过此值后 track 从 PREDICTING 进入 LOST。
    // 增大可容忍更长暂短遮挡，但增加 LOST 状态判定延迟。
    pnh.param<double>("max_predict_time", max_predict_time, 0.30);
    // 丢失后允许预测的最大时长 (s)。在此时长内使用速度预测位姿 (state=PREDICTING)，
    // 超过后进入 LOST。需小于 track_retention_time。增大可容忍更长遮挡。
    pnh.param<double>("track_retention_time", track_retention_time, 1.0);
    // LOST 后 track 在 map 中保留的时长 (s)。超限后 track 被删除 (tag_tracks.erase)。
    // 需大于 max_predict_time。增大可在重新检测到同一 tag 时复用滤波状态。

    // ── 置信度参数 ──
    pnh.param<double>("confidence_threshold", confidence_threshold, 0.5);
    // 置信度有效阈值 [0,1]。confidence 低于此值时 pose_valid=false。
    // 降低阈值可增加有效输出率但引入更多不可靠估计；升高则相反。
    pnh.param<double>("tau_lost", tau_lost, 0.2);
    // 丢失后置信度指数衰减时间常数 (s)。c_lost = exp(-lost_time/tau_lost)。
    // 越小衰减越快，丢失后置信度迅速降为零；增大则容忍更长的丢失期间。
    pnh.param<double>("tau_detection", tau_detection, 0.15);
    // 检测延迟置信度指数衰减时间常数 (s)。c_age = exp(-detection_age/tau_detection)。
    // 越小对检测延迟越敏感；增大则容忍更大检测延迟对置信度的影响。
    pnh.param<double>("residual_scale", residual_scale, 0.5);
    // 残差置信度指数衰减缩放因子 (m)。c_residual = exp(-residual_norm/residual_scale)。
    // 越小对测量残差越敏感 (残差稍大便大幅降低置信度)；增大则更宽容。
    pnh.param<double>("imu_stale_confidence_scale", imu_stale_confidence_scale, 0.35);
    // IMU 超时时置信度乘数 [0,1]。IMU 数据超时后无法做姿态补偿，
    // 使用水平假设导致位置存在系统性偏差，此乘数降低该情况下的置信度。
    // 值越小对 IMU 依赖越强；1.0 表示忽略 IMU 状态。
}

/**
 * @brief 校验参数取值范围，将不合法值重置为默认并输出警告。
 *
 * @note 校验规则按参数分组：正值校验 (rate/timeout) -> 范围校验 (alpha [0,1]) ->
 *       逻辑约束 (max > min, retention >= predict)。
 *       重置后的值保证不会导致运行时除零或越界。
 */
static void validate_parameters() {
    if (runtime_check_rate <= 0.0) {
        ROS_WARN("[TargetEstimator] runtime_check_rate must be positive, reset to 20.0");
        runtime_check_rate = 20.0;
    }
    if (estimator_update_rate <= 0.0) {
        ROS_WARN("[TargetEstimator] estimator_update_rate must be positive, reset to 30.0");
        estimator_update_rate = 30.0;
    }
    if (log_report_rate <= 0.0) {
        ROS_WARN("[TargetEstimator] log_report_rate must be positive, reset to 1.0");
        log_report_rate = 1.0;
    }
    if (vehicle_status_timeout <= 0.0) {
        ROS_WARN("[TargetEstimator] vehicle_status_timeout must be positive, reset to 2.0");
        vehicle_status_timeout = 2.0;
    }
    if (tag_detection_timeout <= 0.0) {
        ROS_WARN("[TargetEstimator] tag_detection_timeout must be positive, reset to 0.5");
        tag_detection_timeout = 0.5;
    }
    if (imu_timeout <= 0.0) {
        ROS_WARN("[TargetEstimator] imu_timeout must be positive, reset to 0.1");
        imu_timeout = 0.1;
    }
    if (min_valid_distance < 0.0) {
        ROS_WARN("[TargetEstimator] min_valid_distance must be non-negative, reset to 0.15");
        min_valid_distance = 0.15;
    }
    if (max_valid_distance <= min_valid_distance) {
        ROS_WARN("[TargetEstimator] max_valid_distance must be greater than min_valid_distance, reset to 10.0");
        max_valid_distance = 10.0;
    }
    if (max_position_jump <= 0.0) {
        ROS_WARN("[TargetEstimator] max_position_jump must be positive, reset to 1.0");
        max_position_jump = 1.0;
    }
    alpha = clamp(alpha, 0.0, 1.0);
    if (beta < 0.0) {
        ROS_WARN("[TargetEstimator] beta must be non-negative, reset to 0.08");
        beta = 0.08;
    }
    if (min_filter_dt < 0.0) {
        ROS_WARN("[TargetEstimator] min_filter_dt must be non-negative, reset to 0.005");
        min_filter_dt = 0.005;
    }
    if (max_filter_dt <= min_filter_dt) {
        ROS_WARN("[TargetEstimator] max_filter_dt must be greater than min_filter_dt, reset to 0.20");
        max_filter_dt = 0.20;
    }
    if (stable_frame_threshold < 1) {
        ROS_WARN("[TargetEstimator] stable_frame_threshold must be >= 1, reset to 3");
        stable_frame_threshold = 3;
    }
    if (lost_frame_threshold < 1) {
        ROS_WARN("[TargetEstimator] lost_frame_threshold must be >= 1, reset to 5");
        lost_frame_threshold = 5;
    }
    if (max_predict_time < 0.0) {
        ROS_WARN("[TargetEstimator] max_predict_time must be non-negative, reset to 0.30");
        max_predict_time = 0.30;
    }
    if (track_retention_time < max_predict_time) {
        // track_retention_time 必须大于 max_predict_time，否则 LOST 后 track
        // 会在预测期未结束前就被删除，导致 PREDICTING->LOST->erase 跳变。
        ROS_WARN("[TargetEstimator] track_retention_time must be >= max_predict_time, reset to 1.0");
        track_retention_time = 1.0;
    }
    confidence_threshold = clamp(confidence_threshold, 0.0, 1.0);
    if (tau_lost <= 0.0) {
        ROS_WARN("[TargetEstimator] tau_lost must be positive, reset to 0.2");
        tau_lost = 0.2;
    }
    if (tau_detection <= 0.0) {
        ROS_WARN("[TargetEstimator] tau_detection must be positive, reset to 0.15");
        tau_detection = 0.15;
    }
    if (residual_scale <= 0.0) {
        ROS_WARN("[TargetEstimator] residual_scale must be positive, reset to 0.5");
        residual_scale = 0.5;
    }
    imu_stale_confidence_scale = clamp(imu_stale_confidence_scale, 0.0, 1.0);
}

/**
 * @brief 输出启动参数汇总日志。
 *
 * @note 在 load_parameters/validate_parameters 之后调用，确认实际生效参数值。
 */
static void print_startup_info() {
    ROS_INFO("[TargetEstimator] running");
    ROS_INFO("[TargetEstimator] tag_detection_topic=%s vehicle_status_topic=%s imu_topic=%s",
             tag_detection_topic.c_str(), vehicle_status_topic.c_str(), imu_topic.c_str());
    ROS_INFO("[TargetEstimator] camera_frame=%s body_frame=%s level_frame=%s use_tf_extrinsic=%s",
             camera_frame.c_str(), body_frame.c_str(), level_frame.c_str(), use_tf_extrinsic ? "true" : "false");
    if (!use_tf_extrinsic) {
        ROS_INFO("[TargetEstimator] manual extrinsic xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) rad",
                 camera_translation_x, camera_translation_y, camera_translation_z,
                 camera_roll, camera_pitch, camera_yaw);
    }
    ROS_INFO("[TargetEstimator] alpha=%.3f beta=%.3f imu_timeout=%.3f confidence_threshold=%.3f",
             alpha, beta, imu_timeout, confidence_threshold);
    ROS_INFO("[TargetEstimator] output=/uav/sensing/apriltag_target log=/uav/log_reports/target_estimator");
}

/**
 * @brief AprilTag 检测结果回调：缓存最新检测数据和时间戳。
 * @param[in] msg AprilTag 检测数组消息。
 *
 * @note 只缓存，不处理。检测数据在 estimator_update_timer_cb 中统一消费。
 *       时间戳优先使用消息 header.stamp，为零时回退到 ros::Time::now()。
 */
static void tag_detection_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    const ros::Time recv_time = ros::Time::now();
    latest_detections = *msg;
    tag_detection_received = true;
    last_tag_detection_time = msg->header.stamp.isZero() ? recv_time : msg->header.stamp;
    last_tag_detection_recv_time = recv_time;
    latest_detection_sequence += 1;
    latest_source_frame = msg->header.frame_id;
}

/**
 * @brief 车辆状态回调：缓存 connect_manager 发布的连接状态。
 * @param[in] msg 车辆状态消息。
 *
 * @note 用于判断可靠连接后再进入 NORMAL 处理 tag 检测。
 */
static void vehicle_status_cb(const lite_quad_uav::VehicleStatus::ConstPtr& msg) {
    const ros::Time recv_time = ros::Time::now();
    vehicle_status = *msg;
    vehicle_status_received = true;
    last_vehicle_status_time = msg->header.stamp.isZero() ? recv_time : msg->header.stamp;
    last_vehicle_status_recv_time = recv_time;
}

/**
 * @brief IMU 姿态数据回调：缓存最新 IMU 四元数。
 * @param[in] msg IMU 消息。
 *
 * @note 用于 compensate_body_to_level() 中将机体系位姿补偿到水平体系。
 */
static void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    const ros::Time recv_time = ros::Time::now();
    latest_imu = *msg;
    imu_received = true;
    last_imu_time = msg->header.stamp.isZero() ? recv_time : msg->header.stamp;
    last_imu_recv_time = recv_time;
}

/**
 * @brief 统一刷新运行时健康标志。
 * @param[in] now 当前时间。
 *
 * @note 回调只缓存原始数据和时间戳；所有"当前是否有效"的判断在这里集中计算。
 *       tag_detection_fresh 使用 tag_detection_timeout 判定；
 *       vehicle_connected 要求 vehicle_status 新鲜且 connected=true。
 */
static void update_runtime_check_flags(const ros::Time& now) {
    runtime_flags.vehicle_status_fresh =
        vehicle_status_received && age_sec(now, last_vehicle_status_recv_time) <= vehicle_status_timeout;
    runtime_flags.vehicle_connected = runtime_flags.vehicle_status_fresh && vehicle_status.connected;
    runtime_flags.reliable_vehicle_connection =
        runtime_flags.vehicle_status_fresh &&
        runtime_flags.vehicle_connected;

    runtime_flags.tag_detection_received = tag_detection_received;
    runtime_flags.tag_detection_fresh =
        tag_detection_received && age_sec(now, last_tag_detection_recv_time) <= tag_detection_timeout;
}

/**
 * @brief 更新节点运行状态机。
 * @param[in] now 当前时间。
 *
 * @note INITIALIZING -> NORMAL: 可靠车辆连接建立。
 *       NORMAL -> REINITIALIZING: 可靠车辆连接丢失。
 *       REINITIALIZING -> NORMAL: 可靠车辆连接恢复。
 */
static void update_node_state(const ros::Time& /*now*/) {
    switch (node_state) {
        case TargetEstimatorNodeState::INITIALIZING:
            if (runtime_flags.reliable_vehicle_connection) {
                transition_to(TargetEstimatorNodeState::NORMAL);
            }
            break;
        case TargetEstimatorNodeState::NORMAL:
            if (!runtime_flags.reliable_vehicle_connection) {
                transition_to(TargetEstimatorNodeState::REINITIALIZING);
            }
            break;
        case TargetEstimatorNodeState::REINITIALIZING:
            if (runtime_flags.reliable_vehicle_connection) {
                transition_to(TargetEstimatorNodeState::NORMAL);
            }
            break;
    }
}

/**
 * @brief 刷新估计器运行上下文：先更新健康标志，再更新节点状态。
 * @param[in] now 当前时间。
 */
static void refresh_estimator_context(const ros::Time& now) {
    update_runtime_check_flags(now);
    update_node_state(now);
}

/**
 * @brief 处理最新一批 tag 检测结果：坐标系转换、距离校验、滤波更新、置信度计算。
 * @param[in] now 当前时间。
 *
 * @note 主处理函数，由 estimator_update_timer_cb 在 NORMAL 状态下调用。
 *       流程：获取外参变换 -> 遍历检测 -> 坐标转换 -> 距离校验 -> 滤波 -> 状态/置信度更新。
 */
static void process_latest_detections(const ros::Time& now) {
    latest_detected_tag_ids.clear();
    std::set<int> detected_ids;
    tf2::Transform tf_body_camera;
    const bool use_current_imu_attitude = imu_attitude_available(now);
    const bool imu_available = imu_received;
    std::string frame_warning_source;

    if (!get_body_to_camera_transform(last_tag_detection_recv_time, &tf_body_camera)) {
        update_missing_tracks(detected_ids, now);
        last_message = "body to camera transform unavailable";
        return;
    }

    for (const auto& detection : latest_detections.detections) {
        if (detection.id.empty()) {
            continue;
        }

        const int tag_id = detection.id.front();
        latest_detected_tag_ids.push_back(tag_id);

        const std::string detection_frame =
            detection.pose.header.frame_id.empty() ? latest_detections.header.frame_id : detection.pose.header.frame_id;
        if (!detection_frame.empty() && detection_frame != camera_frame) {
            frame_warning_source = detection_frame;
            ROS_WARN_THROTTLE(1.0,
                              "[TargetEstimator] detection frame '%s' differs from configured camera_frame '%s'",
                              detection_frame.c_str(), camera_frame.c_str());
        }

        const ros::Time detection_stamp =
            detection.pose.header.stamp.isZero()
                ? (latest_detections.header.stamp.isZero() ? last_tag_detection_recv_time : latest_detections.header.stamp)
                : detection.pose.header.stamp;
        const double recv_age = age_sec(now, last_tag_detection_recv_time);
        const double detection_age =
            age_sec_or_fallback(now, detection_stamp, recv_age, "tag detection stamp");
        if (detection_age > tag_detection_timeout) {
            ROS_WARN_THROTTLE(1.0,
                              "[TargetEstimator] tag detection id=%d header age=%.3f, using receive-time freshness",
                              tag_id, detection_age);
        }

        detected_ids.insert(tag_id);

        TagTrack& track = tag_tracks[tag_id];
        track.tag_id = tag_id;
        track.detected = true;
        track.detection_age = detection_age;
        track.imu_age = current_imu_age(now);
        track.last_update_time = now;

        // 同一帧检测消息可能因定时器间隔短而被重复处理，用接收序号去重。
        if (track.last_processed_detection_sequence == latest_detection_sequence) {
            const double distance = point_norm(track.position_level_raw);
            const bool distance_valid = distance >= min_valid_distance && distance <= max_valid_distance;
            const bool accepted = track.filter_initialized && track.state != "REJECTED";
            track.confidence =
                compute_detection_confidence(track, accepted, distance_valid, use_current_imu_attitude);
            track.pose_valid = track.confidence >= confidence_threshold;
            continue;
        }

        tf2::Transform tf_camera_tag = pose_msg_to_tf(detection.pose.pose.pose);
        tf2::Transform tf_body_tag = tf_body_camera * tf_camera_tag;
        geometry_msgs::Pose measured_pose = tf_to_pose_msg(tf_body_tag);
        geometry_msgs::Point position_level_raw = compensate_body_to_level(measured_pose.position, use_current_imu_attitude);

        track.pose_body_raw = measured_pose;
        track.position_level_raw = position_level_raw;
        track.has_pose = true;
        track.last_seen_time = detection_stamp;
        track.last_seen_recv_time = last_tag_detection_recv_time;
        track.last_processed_detection_stamp = detection_stamp;
        track.last_processed_detection_sequence = latest_detection_sequence;

        const double distance = point_norm(position_level_raw);
        const bool distance_valid = distance >= min_valid_distance && distance <= max_valid_distance;

        if (distance_valid && update_track_filter(track, position_level_raw, detection_stamp, last_tag_detection_recv_time)) {
            track.lost_count = 0;
            // stable_count 达到阈值时升级为 TRACKING，否则停留在 DETECTED。
            track.state = track.stable_count >= static_cast<uint32_t>(stable_frame_threshold)
                              ? "TRACKING"
                              : "DETECTED";
            track.confidence = compute_detection_confidence(track, true, true, use_current_imu_attitude);
            track.pose_valid = track.confidence >= confidence_threshold;
        } else {
            track.stable_count = 0;
            track.confidence = 0.0;
            track.pose_valid = false;
            if (!track.filter_initialized) {
                track.position_level_filtered = position_level_raw;
            }
            track.state = distance_valid ? "REJECTED" : "DETECTED";
            const std::string reject_reason = distance_valid ? "measurement jump rejected" : "distance out of range";
            last_message = reject_reason;
            ROS_WARN_THROTTLE(1.0, "[TargetEstimator] tag %d rejected: %s residual=%.3f distance=%.3f",
                              tag_id, reject_reason.c_str(), track.residual_norm, distance);
        }
    }

    update_missing_tracks(detected_ids, now);
    if (!latest_detected_tag_ids.empty()) {
        if (!imu_available) {
            last_message = "imu unavailable, using level assumption";
        } else if (!use_current_imu_attitude) {
            last_message = "imu timeout, using level assumption";
        } else if (!frame_warning_source.empty()) {
            last_message = "tag detections processed with frame warning";
        } else {
            last_message = "tag detections processed";
        }
    } else {
        last_message = "no tag detected";
    }
}

/**
 * @brief 更新未在当前检测中出现的 track：预测外推或标记 LOST。
 * @param[in] detected_ids 当前周期检测到的 tag ID 集合。
 * @param[in] now 当前时间。
 *
 * @note 丢失未超 max_predict_time 且 lost_count 未超 lost_frame_threshold 时，
 *       使用速度预测位姿 (PREDICTING)；否则标记 LOST。
 *       超过 track_retention_time 的 LOST track 从 map 中删除。
 */
static void update_missing_tracks(const std::set<int>& detected_ids, const ros::Time& now) {
    for (auto it = tag_tracks.begin(); it != tag_tracks.end();) {
        TagTrack& track = it->second;
        if (detected_ids.find(track.tag_id) != detected_ids.end()) {
            ++it;
            continue;
        }

        track.detected = false;
        track.last_update_time = now;
        track.lost_count += 1;
        track.stable_count = 0;

        const double lost_time =
            track.last_seen_recv_time.isZero() ? track_retention_time + 1.0 : age_sec(now, track.last_seen_recv_time);
        // 丢失时间未超 max_predict_time 且 lost_count 未超阈值时，使用速度外推预测。
        if (track.filter_initialized &&
            lost_time <= max_predict_time &&
            track.lost_count <= static_cast<uint32_t>(lost_frame_threshold)) {
            const double predict_dt = std::max(0.0, age_sec(now, track.last_filter_recv_time));
            track.position_level_filtered =
                add_velocity_dt(track.position_level_filtered, track.velocity_level_filtered, predict_dt);
            track.last_filter_recv_time = now;
            track.state = "PREDICTING";
            const bool use_current_imu_attitude = imu_attitude_available(now);
            const double imu_factor = use_current_imu_attitude ? 1.0 : imu_stale_confidence_scale;
            track.imu_age = current_imu_age(now);
            track.confidence = clamp(lost_confidence(lost_time) * imu_factor, 0.0, 1.0);
            track.pose_valid = track.confidence >= confidence_threshold;
            ++it;
            continue;
        }

        track.state = "LOST";
        track.pose_valid = false;
        track.confidence = 0.0;

        if (lost_time > track_retention_time) {
            it = tag_tracks.erase(it);
        } else {
            ++it;
        }
    }
}

/**
 * @brief 将所有 track 标记为无效 (连接丢失或节点未就绪时调用)。
 * @param[in] message 状态描述。
 * @param[in] now 当前时间。
 */
static void mark_all_tracks_invalid(const std::string& message, const ros::Time& now) {
    latest_detected_tag_ids.clear();
    for (auto& item : tag_tracks) {
        TagTrack& track = item.second;
        track.detected = false;
        track.pose_valid = false;
        track.confidence = 0.0;
        track.state = "LOST";
        track.last_update_time = now;
    }
    last_message = message;
}

/**
 * @brief 获取机体系到相机系的变换矩阵。
 * @param[in] stamp 查询时间戳 (用于 TF 插值)。
 * @param[out] tf_body_camera 输出变换矩阵 (body -> camera)。
 * @return true=获取成功。
 *
 * @note use_tf_extrinsic=true 时从 TF 树查询 body_frame->camera_frame 变换；
 *       use_tf_extrinsic=false 时使用手动外参参数 (camera_translation_x/y/z, camera_roll/pitch/yaw)。
 */
static bool get_body_to_camera_transform(const ros::Time& stamp, tf2::Transform* tf_body_camera) {
    if (tf_body_camera == nullptr) {
        return false;
    }

    if (!use_tf_extrinsic) {
        tf2::Quaternion q;
        q.setRPY(camera_roll, camera_pitch, camera_yaw);
        q.normalize();
        tf_body_camera->setOrigin(tf2::Vector3(camera_translation_x, camera_translation_y, camera_translation_z));
        tf_body_camera->setRotation(q);
        return true;
    }

    if (!tf_buffer) {
        ROS_WARN_THROTTLE(1.0, "[TargetEstimator] tf buffer not initialized");
        return false;
    }

    try {
        geometry_msgs::TransformStamped transform =
            tf_buffer->lookupTransform(body_frame, camera_frame, stamp, ros::Duration(0.02));
        tf2::fromMsg(transform.transform, *tf_body_camera);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[TargetEstimator] TF lookup %s -> %s failed: %s",
                          body_frame.c_str(), camera_frame.c_str(), ex.what());
        return false;
    }
}

/**
 * @brief 将 geometry_msgs::Pose 转换为 tf2::Transform。
 * @param[in] pose ROS Pose 消息。
 * @return tf2 变换矩阵。
 */
static tf2::Transform pose_msg_to_tf(const geometry_msgs::Pose& pose) {
    tf2::Transform transform;
    tf2::fromMsg(pose, transform);
    return transform;
}

/**
 * @brief 将 tf2::Transform 转换为 geometry_msgs::Pose。
 * @param[in] transform tf2 变换矩阵。
 * @return ROS Pose 消息。
 */
static geometry_msgs::Pose tf_to_pose_msg(const tf2::Transform& transform) {
    geometry_msgs::Pose pose;
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation = tf2::toMsg(transform.getRotation());
    return pose;
}

/**
 * @brief 将 tf2::Vector3 转换为 geometry_msgs::Point。
 * @param[in] vector tf2 三维向量。
 * @return ROS Point 消息。
 */
static geometry_msgs::Point vector_to_point(const tf2::Vector3& vector) {
    geometry_msgs::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

/**
 * @brief 将 geometry_msgs::Point 转换为 tf2::Vector3。
 * @param[in] point ROS Point 消息。
 * @return tf2 三维向量。
 */
static tf2::Vector3 point_to_vector(const geometry_msgs::Point& point) {
    return tf2::Vector3(point.x, point.y, point.z);
}

/**
 * @brief 位置 + 速度 * dt 外推，用于 PREDICTING 状态。
 * @param[in] point 当前位置。
 * @param[in] velocity 当前速度 (m/s)。
 * @param[in] dt 时间步长 (s)。
 * @return 外推后的位置。
 */
static geometry_msgs::Point add_velocity_dt(const geometry_msgs::Point& point,
                                            const geometry_msgs::Vector3& velocity,
                                            double dt) {
    geometry_msgs::Point result = point;
    result.x += velocity.x * dt;
    result.y += velocity.y * dt;
    result.z += velocity.z * dt;
    return result;
}

/**
 * @brief 构造零速度向量。
 * @return 各分量均为 0 的 geometry_msgs::Vector3。
 */
static geometry_msgs::Vector3 zero_vector3() {
    geometry_msgs::Vector3 vector;
    vector.x = 0.0;
    vector.y = 0.0;
    vector.z = 0.0;
    return vector;
}

/**
 * @brief 检查 IMU 姿态数据是否新鲜可用。
 * @param[in] now 当前时间。
 * @return true=IMU 已收到且未超时。
 */
static bool imu_attitude_available(const ros::Time& now) {
    return imu_received && age_sec(now, last_imu_recv_time) <= imu_timeout;
}

/**
 * @brief 计算当前 IMU 数据延迟。
 * @param[in] now 当前时间。
 * @return IMU 数据延迟 (s)，未收到时返回 -1.0。
 */
static double current_imu_age(const ros::Time& now) {
    if (!imu_received) {
        return -1.0;
    }
    return age_sec(now, last_imu_recv_time);
}

/**
 * @brief 将机体系位置补偿到水平体系 (去除 roll/pitch，保留 yaw)。
 * @param[in] body_position 机体系位置。
 * @param[in] use_current_imu_attitude 是否使用当前 IMU 姿态补偿。
 * @return 水平体系位置。
 *
 * @note IMU 不可用时直接返回原始 body_position (水平假设)。
 *       补偿方法：提取 IMU 的 RPY，构造只保留 yaw 的旋转，
 *       计算 q_level_body = q_level^(-1) * q_body，将 body 系位置旋转到 level 系。
 */
static geometry_msgs::Point compensate_body_to_level(const geometry_msgs::Point& body_position,
                                                     bool use_current_imu_attitude) {
    if (!use_current_imu_attitude) {
        return body_position;
    }

    tf2::Quaternion q_imu;
    tf2::fromMsg(latest_imu.orientation, q_imu);
    if (q_imu.length2() <= 1e-12) {
        ROS_WARN_THROTTLE(1.0, "[TargetEstimator] IMU orientation quaternion is invalid, using level assumption");
        return body_position;
    }
    q_imu.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_body;
    q_body.setRPY(roll, pitch, yaw);
    tf2::Quaternion q_level;
    q_level.setRPY(0.0, 0.0, yaw);

    // 构造 body->level 的旋转：去除 roll/pitch 只保留 yaw，
    // 即 q_level_body = q_level^(-1) * q_body，将 body 系位置旋转到 level 系。
    const tf2::Quaternion q_level_body = q_level.inverse() * q_body;
    const tf2::Vector3 p_level = tf2::Matrix3x3(q_level_body) * point_to_vector(body_position);
    return vector_to_point(p_level);
}

/**
 * @brief 使用 alpha-beta 滤波器更新 track 的位置和速度估计。
 * @param[in,out] track 目标跟踪记录。
 * @param[in] measurement 本次水平体系测量位置。
 * @param[in] detection_stamp 检测时间戳。
 * @return true=滤波更新成功；false=测量跳变过大被拒绝。
 *
 * @note 首次调用时直接赋值初始化。后续调用：
 *       1. 计算 dt，截断到 [min_filter_dt, max_filter_dt]
 *       2. 用上一次滤波结果 + 速度预测当前位置
 *       3. 计算测量与预测的残差，超过 max_position_jump 则拒绝
 *       4. 位置修正: filtered = predicted + alpha * residual
 *       5. 速度修正: velocity += beta * residual / dt
 */
static bool update_track_filter(TagTrack& track,
                                const geometry_msgs::Point& measurement,
                                const ros::Time& detection_stamp,
                                const ros::Time& filter_time) {
    if (!track.filter_initialized) {
        track.position_level_filtered = measurement;
        track.velocity_level_filtered = zero_vector3();
        track.filter_initialized = true;
        track.last_filter_time = detection_stamp;
        track.last_filter_recv_time = filter_time;
        track.residual_norm = 0.0;
        track.stable_count = 1;
        return true;
    }

    double dt = 0.0;
    if (!safe_delta_sec(filter_time, track.last_filter_recv_time, &dt, "tag filter dt")) {
        track.position_level_filtered = measurement;
        track.velocity_level_filtered = zero_vector3();
        track.last_filter_time = detection_stamp;
        track.last_filter_recv_time = filter_time;
        track.residual_norm = 0.0;
        track.stable_count = 1;
        return true;
    }
    if (dt < min_filter_dt) {
        return true;
    }
    if (dt > max_filter_dt) {
        dt = max_filter_dt;
    }

    const geometry_msgs::Point predicted =
        add_velocity_dt(track.position_level_filtered, track.velocity_level_filtered, dt);
    const double rx = measurement.x - predicted.x;
    const double ry = measurement.y - predicted.y;
    const double rz = measurement.z - predicted.z;
    const double residual = std::sqrt(rx * rx + ry * ry + rz * rz);
    track.residual_norm = residual;

    if (residual > max_position_jump) {
        return false;
    }

    track.position_level_filtered.x = predicted.x + alpha * rx;
    track.position_level_filtered.y = predicted.y + alpha * ry;
    track.position_level_filtered.z = predicted.z + alpha * rz;

    const double safe_dt = std::max(dt, 1e-6);
    track.velocity_level_filtered.x += beta * rx / safe_dt;
    track.velocity_level_filtered.y += beta * ry / safe_dt;
    track.velocity_level_filtered.z += beta * rz / safe_dt;

    track.last_filter_time = detection_stamp;
    track.last_filter_recv_time = filter_time;
    track.stable_count += 1;
    return true;
}

/**
 * @brief 计算检测的综合置信度。
 * @param[in] track 目标跟踪记录。
 * @param[in] accepted 本次检测是否被接受 (未跳变/距离有效)。
 * @param[in] distance_valid 距离是否在有效范围内。
 * @param[in] use_current_imu_attitude IMU 姿态是否可用。
 * @return 综合置信度 [0,1]。
 *
 * @note 置信度 = c_detect * c_age * c_dist * c_residual * c_stable * c_imu，
 *       各因子独立计算后相乘，最终 clamp 到 [0,1]。
 *       - c_detect: 基础检测因子 (固定 1.0)
 *       - c_age: 检测延迟衰减 exp(-age/tau_detection)
 *       - c_dist: 距离衰减，越远越低 [0.2, 1.0]
 *       - c_residual: 残差衰减 exp(-residual/residual_scale)
 *       - c_stable: 稳定帧计数比 stable_count/stable_frame_threshold
 *       - c_imu: IMU 可用时为 1.0，超时时为 imu_stale_confidence_scale
 */
static double compute_detection_confidence(const TagTrack& track,
                                           bool accepted,
                                           bool distance_valid,
                                           bool use_current_imu_attitude) {
    if (!accepted || !distance_valid) {
        return 0.0;
    }

    const double distance = point_norm(track.position_level_raw);
    const double c_detect = 1.0;
    const double c_age = std::exp(-track.detection_age / std::max(tau_detection, 1e-6));
    const double c_dist = distance_valid
                              ? clamp(1.0 - distance / std::max(max_valid_distance, 1e-6), 0.2, 1.0)
                              : 0.0;
    const double c_residual = std::exp(-track.residual_norm / std::max(residual_scale, 1e-6));
    const double c_stable = stable_confidence(track.stable_count);
    const double c_imu = use_current_imu_attitude ? 1.0 : imu_stale_confidence_scale;
    return clamp(c_detect * c_age * c_dist * c_residual * c_stable * c_imu, 0.0, 1.0);
}

/**
 * @brief 计算 Point 的欧几里得范数。
 * @param[in] point 三维点。
 * @return 距离原点的距离 (m)。
 */
static double point_norm(const geometry_msgs::Point& point) {
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

/**
 * @brief 将值限制在 [min_value, max_value] 范围内。
 * @param[in] value 输入值。
 * @param[in] min_value 下限。
 * @param[in] max_value 上限。
 * @return 限制后的值。
 */
static double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

/**
 * @brief 计算从 stamp 到 now 的时间差。
 * @param[in] now 当前时间。
 * @param[in] stamp 目标时间戳。
 * @return 时间差 (s)，stamp 为零或不可比较时返回 infinity。
 */
static double age_sec(const ros::Time& now, const ros::Time& stamp) {
    double age = 0.0;
    if (!safe_delta_sec(now, stamp, &age, "age")) {
        return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, age);
}

/**
 * @brief 计算 age，若 header stamp 与当前 ROS 时间不可比较则使用 fallback。
 * @param[in] now 当前时间。
 * @param[in] stamp 待计算的源消息时间戳。
 * @param[in] fallback_age 接收时间推导出的 fallback age。
 * @param[in] label 日志上下文。
 * @return 可用于显示/置信度计算的 age。
 */
static double age_sec_or_fallback(const ros::Time& now,
                                  const ros::Time& stamp,
                                  double fallback_age,
                                  const char* label) {
    double age = 0.0;
    if (!safe_delta_sec(now, stamp, &age, label)) {
        return fallback_age;
    }
    return age;
}

/**
 * @brief 安全计算两个 ROS 时间戳的差值。
 * @param[in] end 结束时间。
 * @param[in] start 起始时间。
 * @param[out] dt 输出秒数。
 * @param[in] label 日志上下文。
 * @return true=时间戳可比较；false=时间戳为空、跨时间域或发生异常。
 */
static bool safe_delta_sec(const ros::Time& end,
                           const ros::Time& start,
                           double* dt,
                           const char* label) {
    if (dt == nullptr || end.isZero() || start.isZero()) {
        return false;
    }

    double value = 0.0;
    try {
        value = (end - start).toSec();
    } catch (const std::runtime_error& ex) {
        ROS_WARN_THROTTLE(1.0,
                          "[TargetEstimator] %s time delta is invalid: %s",
                          label == nullptr ? "unknown" : label,
                          ex.what());
        return false;
    }

    if (!std::isfinite(value)) {
        ROS_WARN_THROTTLE(1.0,
                          "[TargetEstimator] %s time delta is not finite",
                          label == nullptr ? "unknown" : label);
        return false;
    }

    // 大幅负值或超大正值通常意味着 wall time / sim time 混用。
    if (value < -0.05 || value > 60.0) {
        ROS_WARN_THROTTLE(1.0,
                          "[TargetEstimator] %s time delta %.3f is out of expected range",
                          label == nullptr ? "unknown" : label,
                          value);
        return false;
    }

    *dt = std::max(0.0, value);
    return true;
}

/**
 * @brief 计算稳定帧计数置信度因子。
 * @param[in] stable_count 连续稳定检测帧数。
 * @return 置信度因子 [0,1]，= stable_count / stable_frame_threshold。
 */
static double stable_confidence(uint32_t stable_count) {
    const double threshold = std::max(1, stable_frame_threshold);
    return clamp(static_cast<double>(stable_count) / threshold, 0.0, 1.0);
}

/**
 * @brief 计算丢失时间置信度因子 (指数衰减)。
 * @param[in] lost_time 丢失时长 (s)。
 * @return 置信度因子 [0,1]，= exp(-lost_time/tau_lost)。
 */
static double lost_confidence(double lost_time) {
    return clamp(std::exp(-lost_time / std::max(tau_lost, 1e-6)), 0.0, 1.0);
}

/**
 * @brief 发布 AprilTagTargetArray 消息，包含所有 track 状态。
 * @param[in] now 当前时间。
 */
static void publish_target_array(const ros::Time& now) {
    lite_quad_uav::AprilTagTargetArray msg;
    msg.header.stamp = now;
    msg.header.frame_id = level_frame;
    msg.source_frame = latest_source_frame;
    msg.camera_frame = camera_frame;
    msg.body_frame = body_frame;
    msg.level_frame = level_frame;
    msg.detected_tag_ids = latest_detected_tag_ids;

    for (const auto& item : tag_tracks) {
        msg.targets.push_back(make_target_msg(item.second));
    }

    msg.target_count = msg.targets.size();
    target_pub.publish(msg);
    update_output_counts();
}

/**
 * @brief 从 TagTrack 构造 AprilTagTarget 消息。
 * @param[in] track 目标跟踪记录。
 * @return AprilTagTarget 消息。
 */
static lite_quad_uav::AprilTagTarget make_target_msg(const TagTrack& track) {
    lite_quad_uav::AprilTagTarget msg;
    msg.tag_id = track.tag_id;
    msg.state = track.state;
    msg.detected = track.detected;
    msg.pose_valid = track.pose_valid;
    msg.pose_body_raw = track.pose_body_raw;
    msg.position_level_raw = track.position_level_raw;
    msg.position_level_filtered = track.position_level_filtered;
    msg.confidence = track.confidence;
    msg.detection_age = track.detection_age;
    msg.imu_age = track.imu_age;
    msg.residual_norm = track.residual_norm;
    msg.stable_count = track.stable_count;
    msg.lost_count = track.lost_count;
    return msg;
}

/**
 * @brief 更新输出计数统计 (detected/tracked/valid)，供 log report 使用。
 */
static void update_output_counts() {
    last_detected_count = latest_detected_tag_ids.size();
    last_tracked_count = 0;
    last_valid_count = 0;
    for (const auto& item : tag_tracks) {
        const TagTrack& track = item.second;
        if (track.state == "DETECTED" ||
            track.state == "TRACKING" ||
            track.state == "PREDICTING") {
            last_tracked_count += 1;
        }
        if (track.pose_valid) {
            last_valid_count += 1;
        }
    }
}

/**
 * @brief 运行时健康检查定时器：只刷新标志位，不做状态跳转。
 * @param[in] event 定时器事件。
 */
static void runtime_check_timer_cb(const ros::TimerEvent& /*event*/) {
    update_runtime_check_flags(ros::Time::now());
}

/**
 * @brief 估计器主更新定时器：刷新上下文，根据节点状态处理检测并发布结果。
 * @param[in] event 定时器事件。
 *
 * @note 非 NORMAL 状态下所有 track 标记为 LOST。
 *       tag 检测超时时仅更新丢失 track (不处理最新检测)。
 */
static void estimator_update_timer_cb(const ros::TimerEvent& /*event*/) {
    const ros::Time now = ros::Time::now();
    refresh_estimator_context(now);

    if (node_state != TargetEstimatorNodeState::NORMAL) {
        mark_all_tracks_invalid("waiting for reliable vehicle connection", now);
        publish_target_array(now);
        return;
    }

    if (!runtime_flags.tag_detection_fresh) {
        update_missing_tracks(std::set<int>(), now);
        last_message = runtime_flags.tag_detection_received ? "tag detection timeout" : "waiting for tag detections";
        publish_target_array(now);
        return;
    }

    process_latest_detections(now);
    publish_target_array(now);
}

/**
 * @brief 周期发布估计器日志报告，供 logger_node 集中展示。
 * @param[in] event 定时器事件。
 */
static void log_report_timer_cb(const ros::TimerEvent& /*event*/) {
    const ros::Time now = ros::Time::now();
    refresh_estimator_context(now);
    update_output_counts();

    lite_quad_uav::TargetEstimatorLogReport report;
    report.header.stamp = now;
    report.node_state = node_state_to_string(node_state);
    report.vehicle_status_fresh = runtime_flags.vehicle_status_fresh;
    report.reliable_vehicle_connection = runtime_flags.reliable_vehicle_connection;
    report.tag_detection_received = runtime_flags.tag_detection_received;
    report.tag_detection_fresh = runtime_flags.tag_detection_fresh;
    report.detected_count = last_detected_count;
    report.tracked_count = last_tracked_count;
    report.valid_count = last_valid_count;
    report.source_frame = latest_source_frame;
    report.body_frame = body_frame;
    report.message = last_message;
    log_report_pub.publish(report);
}
