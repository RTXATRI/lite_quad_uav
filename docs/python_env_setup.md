# Python 环境配置指南

## 1. 推荐 Python 版本

ROS Noetic 默认使用 **Python 3**（通常为 Python 3.8）。请确认系统 Python 版本：

```bash
python3 --version
```

ROS Noetic 的 `rospy` 基于 Python 3，不需要额外安装。

## 2. ROS Noetic 下 rospy 使用说明

`rospy` 随 ROS Noetic 自动安装在 `/opt/ros/noetic/lib/python3/dist-packages/`。

使用前需要 source ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/code/catkin_ws/devel/setup.bash
```

`rospy` 不需要 pip 安装，也不应该用 pip 安装。

## 3. 是否需要创建 virtualenv / conda 环境

**不需要。**

ROS1 的 Python 节点必须使用系统 Python 3 和 ROS 自带的 `rospy`。使用 virtualenv 或 conda 环境会导致 `rospy` 无法导入。

## 4. 如果使用 conda，如何避免影响 ROS Python 环境

如果系统已经安装了 conda，需要确保 conda 不会覆盖系统 Python：

```bash
# 方法1：关闭 conda 自动激活
conda config --set auto_activate_base false

# 方法2：在运行 ROS 节点前手动 deactivate
conda deactivate

# 方法3：确保 ROS source 在 conda deactivate 之后
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/code/catkin_ws/devel/setup.bash
```

关键原则：**运行 ROS Python 节点时，必须使用系统 Python 3，不能使用 conda 的 Python。**

## 5. Python 节点依赖说明

当前 Python 节点的依赖极少：

| 节点 | 依赖 |
|------|------|
| `gcs_bridge_node.py` | 仅 rospy（标准库 + rospy） |
| `logger_node.py` | rospy + lite_quad_uav.msg（catkin 编译后自动生成） |
| `target_detector_node.py` | 仅 rospy（占位节点） |

**不需要额外的 pip 包。**

## 6. pip 安装命令

当前阶段不需要安装任何 pip 包。所有占位节点仅使用 Python 标准库和 `rospy`。

如果后续需要扩展（如上位机通信、数据处理），可能需要的包：

```bash
# 仅在后续扩展时安装
pip3 install numpy       # 数据处理
pip3 install pyyaml      # YAML 解析（通常已随 ROS 安装）
```

## 7. 检查 Python 节点 shebang 和执行权限

每个 Python 节点文件第一行必须是：

```python
#!/usr/bin/env python
```

检查和设置执行权限：

```bash
# 检查权限
ls -la ~/code/catkin_ws/src/sci_uav/lite_quad_uav/scripts/

# 添加执行权限
chmod +x ~/code/catkin_ws/src/sci_uav/lite_quad_uav/scripts/*.py
```

如果缺少执行权限，`rosrun` 和 `roslaunch` 将无法启动 Python 节点。

## 8. 配置好环境后的编译和运行步骤

```bash
# 1. Source 环境
source /opt/ros/noetic/setup.bash

# 2. 编译
cd ~/code/catkin_ws
catkin_make

# 3. Source 工作空间
source devel/setup.bash

# 4. 先启动 PX4 SITL（另开终端）
cd ~/PX4-Autopilot
make px4_sitl gazebo

# 5. 启动 MAVROS（另开终端）
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# 6. 启动 square_patrol 任务（另开终端）
source ~/code/catkin_ws/devel/setup.bash
roslaunch lite_quad_uav square_patrol.launch
```
