cmake_minimum_required(VERSION 2.8.3)
project(mav_state_estimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/mav_state_estimator.cc
  src/initialization.cc
)
target_link_libraries(${PROJECT_NAME} gtsam)

############
# BINARIES #
############
cs_add_executable(mav_state_estimator src/mav_state_estimator.cc)
target_link_libraries(mav_state_estimator ${PROJECT_NAME})

cs_install()
cs_export()
