cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_arm64.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

# for spot model.
add_executable(example_stop examples/example_stop.cpp)
target_link_libraries(example_stop ${EXTRA_LIBS})

# one pc multi process
add_executable(lcm_server_high examples/lcm_server_high.cpp)
target_link_libraries(lcm_server_high ${EXTRA_LIBS})

add_executable(lcm_server_low examples/lcm_server_low.cpp)
target_link_libraries(lcm_server_low ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})

# lcm server
add_executable(lcm_server examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})

add_executable(a1_pose_once examples/a1_pose_once.cpp)
target_link_libraries(a1_pose_once ${EXTRA_LIBS})

add_executable(a1_mode_test_8080 examples/a1_mode_test_8080.cpp)
target_link_libraries(a1_mode_test_8080 ${EXTRA_LIBS})

add_executable(a1_height_loop examples/a1_height_loop.cpp)
target_link_libraries(a1_height_loop ${EXTRA_LIBS})
add_executable(a1_height_mode1_native examples/a1_height_mode1_native.cpp)
target_link_libraries(a1_height_mode1_native ${EXTRA_LIBS})

add_executable(a1_height_sweep_mode1 examples/a1_height_sweep_mode1.cpp)
target_link_libraries(a1_height_sweep_mode1 ${EXTRA_LIBS})

add_executable(a1_low_readonly examples/a1_low_readonly.cpp)
target_link_libraries(a1_low_readonly ${EXTRA_LIBS})

add_executable(a1_low_probe examples/a1_low_probe.cpp)
target_link_libraries(a1_low_probe ${EXTRA_LIBS})

add_executable(a1_low_takeover_hold examples/a1_low_takeover_hold.cpp)
target_link_libraries(a1_low_takeover_hold ${EXTRA_LIBS})

add_executable(a1_low_takeover_hold2 examples/a1_low_takeover_hold2.cpp)
target_link_libraries(a1_low_takeover_hold2 ${EXTRA_LIBS})

add_executable(a1_low_raise_small examples/a1_low_raise_small.cpp)
target_link_libraries(a1_low_raise_small ${EXTRA_LIBS})

add_executable(a1_low_ready_cycle examples/a1_low_ready_cycle.cpp)
target_link_libraries(a1_low_ready_cycle ${EXTRA_LIBS})

add_executable(a1_low_ready_wait_trigger examples/a1_low_ready_wait_trigger.cpp)
target_link_libraries(a1_low_ready_wait_trigger ${EXTRA_LIBS})

add_executable(a1_low_action_safety_fsm examples/a1_low_action_safety_fsm.cpp)
target_link_libraries(a1_low_action_safety_fsm ${EXTRA_LIBS})

add_executable(a1_high_follow_driver examples/a1_high_follow_driver.cpp)
target_link_libraries(a1_high_follow_driver ${EXTRA_LIBS})

add_executable(a1_high_follow_driver_v2 examples/a1_high_follow_driver_v2.cpp)
target_link_libraries(a1_high_follow_driver_v2 ${EXTRA_LIBS})



