#include "mav_state_estimation/mav_state_estimator.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::spin();
  return 0;
}
