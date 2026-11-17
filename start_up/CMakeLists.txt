cmake_minimum_required(VERSION 3.0.2)
project(start_up)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs std_msgs)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs nav_msgs std_msgs)
set(CMAKE_CXX_STANDARD 17)
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_executable(stage1_cloud_gate_node src/stage1_cloud_gate.cpp)
target_link_libraries(stage1_cloud_gate_node ${catkin_LIBRARIES} yaml-cpp)
add_library(stage1_stamped_truth SHARED src/stage1_stamped_truth.cpp)
target_link_libraries(stage1_stamped_truth ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} yaml-cpp)
install(TARGETS stage1_cloud_gate_node stage1_stamped_truth
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/stage1_sensor_tf.py
  scripts/stage1_cloud_gate.py
  scripts/stage1_ground_truth.py
  scripts/stage1_monitor.py
  scripts/stage1_record.py
  scripts/stage1_evaluate.py
  scripts/stage1_suite.py
  scripts/stage1_ugv_guard.py
  scripts/stage1_uav_teleop.py
  scripts/stage3_gazebo_frontend.py
  scripts/apply_lio_scan_patch.py
  scripts/stage3_record.py
  scripts/stage3_phase.py
  scripts/stage3_evaluate.py
  scripts/stage1_real_exec.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rviz models worlds patches docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/setup_stage1.bash DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
