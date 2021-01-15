cmake_minimum_required(VERSION 2.8.3)
project(mav_state_estimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_python_setup()
catkin_simple(ALL_DEPS_REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_nodelet
  src/absolute_position_factor.cc
  src/initialization.cc
  src/mav_state_estimator.cc
  src/mav_state_estimator_nodelet.cc
  src/moving_baseline_factor.cc
)
target_link_libraries(${PROJECT_NAME}_nodelet gtsam gtsam_unstable)

#########
# TESTS #
#########
catkin_add_gtest(test_absolute_position_factor
  test/absolute_position_factor-test.cc
)
target_link_libraries(test_absolute_position_factor ${PROJECT_NAME}_nodelet)

catkin_add_gtest(test_moving_baseline_factor
  test/moving_baseline_factor-test.cc
)
target_link_libraries(test_moving_baseline_factor ${PROJECT_NAME}_nodelet)

############
# BINARIES #
############
cs_add_executable(mav_state_estimator src/mav_state_estimator_node.cc)
target_link_libraries(mav_state_estimator ${PROJECT_NAME}_nodelet)

catkin_install_python(PROGRAMS nodes/bag_to_csv scripts/export_csv scripts/filter_imu
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()
cs_export()
