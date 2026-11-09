/**
 * @file controller_lib.hpp
 * @brief 控制器库聚合头：PID 控制器与 RL 输出缓存接口。
 *
 * controller_node 只调度控制器；具体控制算法由本控制器库提供。
 *
 * 职责补充：为控制器调度节点提供统一包含入口，集中引入公共类型、PID 实现声明和 RL 缓存接口。
 */

#pragma once

#include <lite_quad_uav/controllers/controller_types.hpp>
#include <lite_quad_uav/controllers/pid_position_controller.hpp>
#include <lite_quad_uav/controllers/rl_interfaces.hpp>
