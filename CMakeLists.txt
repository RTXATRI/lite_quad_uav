cmake_minimum_required(VERSION 3.0.2)
project(lite_quad_uav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

## Generate messages
add_message_files(
  FILES
  AprilTagTarget.msg
  AprilTagTargetArray.msg
  ControlRequest.msg
  ConnectManagerLogReport.msg
  ControllerLogReport.msg
  ControllerFeedback.msg
  PipelineStatus.msg
  SafetyStatus.msg
  SafetySupervisorLogReport.msg
  TaskManagerLogReport.msg
  TaskManagerStatus.msg
  TaskNodeLogReport.msg
  TargetState.msg
  TargetEstimatorLogReport.msg
  TaskStatus.msg
  VehicleStatus.msg
  VelocityManagerLogReport.msg
  VelocityCommand.msg
)

## Generate services
add_service_files(
  FILES
  ControlTask.srv
  StartTask.srv
  VehicleCommand.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_actions ${PROJECT_NAME}_controller_lib ${PROJECT_NAME}_velocity_pipeline
  CATKIN_DEPENDS apriltag_ros geometry_msgs mavros_msgs message_runtime nav_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Libraries (prefixed to avoid workspace-wide target name collisions)
add_library(${PROJECT_NAME}_actions
  src/libraries/actions/actions.cpp
)
add_dependencies(${PROJECT_NAME}_actions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_actions ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_controller_lib
  src/libraries/controllers/rl_interfaces.cpp
  src/libraries/controllers/pid_position_controller.cpp
)
add_dependencies(${PROJECT_NAME}_controller_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_controller_lib ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_velocity_pipeline
  src/libraries/velocity/velocity_pipeline.cpp
)
add_dependencies(${PROJECT_NAME}_velocity_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_velocity_pipeline ${catkin_LIBRARIES})

## Executables (prefixed targets, short output names)
add_executable(${PROJECT_NAME}_connect_manager_node src/core/connect_manager_node.cpp)
set_target_properties(${PROJECT_NAME}_connect_manager_node PROPERTIES OUTPUT_NAME connect_manager_node PREFIX "")
add_dependencies(${PROJECT_NAME}_connect_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_connect_manager_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_safety_supervisor_node src/core/safety_supervisor_node.cpp)
set_target_properties(${PROJECT_NAME}_safety_supervisor_node PROPERTIES OUTPUT_NAME safety_supervisor_node PREFIX "")
add_dependencies(${PROJECT_NAME}_safety_supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_safety_supervisor_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_controller_node src/core/controller_node.cpp)
set_target_properties(${PROJECT_NAME}_controller_node PROPERTIES OUTPUT_NAME controller_node PREFIX "")
add_dependencies(${PROJECT_NAME}_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_controller_node ${PROJECT_NAME}_controller_lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_velocity_manager_node src/core/velocity_manager_node.cpp)
set_target_properties(${PROJECT_NAME}_velocity_manager_node PROPERTIES OUTPUT_NAME velocity_manager_node PREFIX "")
add_dependencies(${PROJECT_NAME}_velocity_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_velocity_manager_node ${PROJECT_NAME}_velocity_pipeline ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_task_manager_node src/core/task_manager_node.cpp)
set_target_properties(${PROJECT_NAME}_task_manager_node PROPERTIES OUTPUT_NAME task_manager_node PREFIX "")
add_dependencies(${PROJECT_NAME}_task_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_task_manager_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_square_patrol_task_node src/tasks/square_patrol_task_node.cpp)
set_target_properties(${PROJECT_NAME}_square_patrol_task_node PROPERTIES OUTPUT_NAME square_patrol_task_node PREFIX "")
add_dependencies(${PROJECT_NAME}_square_patrol_task_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_square_patrol_task_node ${PROJECT_NAME}_actions ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_hover_placeholder_task_node src/tasks/placeholder_task_node.cpp)
set_target_properties(${PROJECT_NAME}_hover_placeholder_task_node PROPERTIES OUTPUT_NAME hover_placeholder_task_node PREFIX "")
target_compile_definitions(${PROJECT_NAME}_hover_placeholder_task_node PRIVATE DEFAULT_TASK_ID=\"hover_placeholder\" DEFAULT_TASK_NS=\"/uav/tasks/hover_placeholder\")
add_dependencies(${PROJECT_NAME}_hover_placeholder_task_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_hover_placeholder_task_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_inspection_placeholder_task_node src/tasks/placeholder_task_node.cpp)
set_target_properties(${PROJECT_NAME}_inspection_placeholder_task_node PROPERTIES OUTPUT_NAME inspection_placeholder_task_node PREFIX "")
target_compile_definitions(${PROJECT_NAME}_inspection_placeholder_task_node PRIVATE DEFAULT_TASK_ID=\"inspection_placeholder\" DEFAULT_TASK_NS=\"/uav/tasks/inspection_placeholder\")
add_dependencies(${PROJECT_NAME}_inspection_placeholder_task_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_inspection_placeholder_task_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_target_estimator_node src/sensing/target_estimator_node.cpp)
set_target_properties(${PROJECT_NAME}_target_estimator_node PROPERTIES OUTPUT_NAME target_estimator_node PREFIX "")
add_dependencies(${PROJECT_NAME}_target_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_target_estimator_node ${catkin_LIBRARIES})

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/gcs_bridge_node.py
  scripts/image_topic_video_recorder.py
  scripts/logger_node.py
  scripts/target_detector_node.py
  scripts/video_camera_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}_actions ${PROJECT_NAME}_controller_lib ${PROJECT_NAME}_velocity_pipeline
  ${PROJECT_NAME}_connect_manager_node ${PROJECT_NAME}_safety_supervisor_node ${PROJECT_NAME}_controller_node
  ${PROJECT_NAME}_velocity_manager_node ${PROJECT_NAME}_task_manager_node ${PROJECT_NAME}_square_patrol_task_node
  ${PROJECT_NAME}_hover_placeholder_task_node ${PROJECT_NAME}_inspection_placeholder_task_node
  ${PROJECT_NAME}_target_estimator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
