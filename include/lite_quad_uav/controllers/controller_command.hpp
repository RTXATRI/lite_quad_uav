/**
 * @file controller_command.hpp
 * @brief 控制器内部命令结构，由 ControlRequest 转换得到，不作为 ROS 消息发布。
 */

#pragma once

#include <cstdint>
#include <string>

#include <std_msgs/Header.h>

/**
 * @brief controller_node 和控制器库内部使用的控制命令。
 */
struct ControllerCommand {
    enum ControllerType : uint8_t {
        CONTROLLER_IDLE = 0,
        CONTROLLER_PID_POSITION = 1,
        CONTROLLER_RL_TRACKING = 2,
        CONTROLLER_RL_LANDING = 3
    };

    enum MixMode : uint8_t {
        MIX_NONE = 0,
        MIX_PID_ONLY = 1,
        MIX_RL_ONLY = 2,
        MIX_PID_RL_TRACKING = 3,
        MIX_PID_RL_LANDING = 4,
        MIX_AUTO = 5
    };

    std_msgs::Header header;
    std::string source_task;
    std::string source_action;

    uint8_t controller_type = CONTROLLER_IDLE;
    uint8_t mix_mode = MIX_NONE;
    bool valid = false;

    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.0;
    double target_yaw = 0.0;
    bool use_yaw = false;

    double max_xy_speed = 0.0;
    double max_z_up_speed = 0.0;
    double max_z_down_speed = 0.0;
    double max_yaw_rate = 0.0;

    double timeout_sec = 0.0;
    std::string message;
};
