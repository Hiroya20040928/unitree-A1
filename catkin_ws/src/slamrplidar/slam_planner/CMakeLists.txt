cmake_minimum_required(VERSION 2.8.3)
project(slam_planner)

find_path(slam_planner_INCLUDE_DIR nav/MoveBaseConfig.h ${PROJECT_SOURCE_DIR}/../slam_planner_sdk/include)
find_path(slam_planner_LIBRARY libunitree_legged_sdk.so ${PROJECT_SOURCE_DIR}/../slam_planner_sdk/lib)
find_path(laikago_sdk_INCLUDE_DIR unitree_legged_sdk/comm.h ${PROJECT_SOURCE_DIR}/../slam_planner_sdk/include)

if(slam_planner_INCLUDE_DIR AND slam_planner_LIBRARY)
  set(SLTC_SDK_INC_DIR "${slam_planner_INCLUDE_DIR}")
  set(SLTC_SDK_LIB_DIR "${slam_planner_LIBRARY}")
 
else(slam_planner_INCLUDE_DIR AND slam_planner_LIBRARY) 
  if(NOT DEFINED SLTC_SDK_ROOT OR SLTC_SDK_ROOT STREQUAL "")
    set(SLTC_SDK_ROOT "/home/pi/catkin_ws/src/slamrplidar/slam_planner_sdk/")
  endif()
#   set(SLTC_SDK_DIR "${SLTC_SDK_ROOT}/linux-x86_64-release")
  set(SLTC_SDK_INC_DIR "${SLTC_SDK_ROOT}/include")
  set(SLTC_SDK_LIB_DIR "${SLTC_SDK_ROOT}/lib")
endif(slam_planner_INCLUDE_DIR AND slam_planner_LIBRARY)

if(laikago_sdk_INCLUDE_DIR)
    set(LKGO_SDK_INC_DIR "${laikago_sdk_INCLUDE_DIR}")
else(laikago_sdk_INCLUDE_DIR)
    set(LKGO_SDK_INC_DIR " ")
endif(laikago_sdk_INCLUDE_DIR)

add_compile_options(-std=c++11)

set(BOOST_ROOT /usr/local/boost1580)
find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs 
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

find_path(REALSENSE2_INCLUDE_DIR
  librealsense2/rs.hpp
  PATHS
    /usr/local/include
    /usr/include
)
find_library(REALSENSE2_LIBRARY
  realsense2
  PATHS
    /usr/local/lib
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib/x86_64-linux-gnu
)

add_message_files(
  FILES
  Cartesian.msg
  HighCmd.msg
  HighState.msg
  IMU.msg
  LED.msg
  LowCmd.msg
  LowState.msg
  MotorCmd.msg
  MotorState.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        message_runtime
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(slam_planner_node
   src/slam_planner.cpp
)
target_link_libraries(slam_planner_node
	${catkin_LIBRARIES}
	${SLTC_SDK_LIB_DIR}/libslam_planner.so
)
add_dependencies(slam_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(slam_planner_node
  PRIVATE ${SLTC_SDK_INC_DIR}
)

add_executable(base_controller_node
    src/slam_base_controller.cpp
)
target_link_libraries(base_controller_node
	${catkin_LIBRARIES}
	/home/unitree/unitree_legged_sdk/lib/libunitree_legged_sdk_arm64.so
    lcm
)
add_dependencies(base_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(base_controller_node BEFORE
  PRIVATE /home/unitree/unitree_legged_sdk/include
)

add_executable(teb_via_teleop_node
    src/teb_via_teleop.cpp
)
target_link_libraries(teb_via_teleop_node
    ${catkin_LIBRARIES}
)
add_dependencies(teb_via_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(REALSENSE2_INCLUDE_DIR AND REALSENSE2_LIBRARY)
  add_executable(realsense_wander_obstacle_avoidance
      src/realsense_wander_obstacle_avoidance.cpp
  )
  target_link_libraries(realsense_wander_obstacle_avoidance
    ${catkin_LIBRARIES}
    ${REALSENSE2_LIBRARY}
  )
  add_dependencies(realsense_wander_obstacle_avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(realsense_wander_obstacle_avoidance
    PRIVATE
      ${REALSENSE2_INCLUDE_DIR}
  )
else()
  message(WARNING "librealsense2 was not found. realsense_wander_obstacle_avoidance will not be built.")
endif()

catkin_install_python(PROGRAMS
  scripts/teb_via_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
