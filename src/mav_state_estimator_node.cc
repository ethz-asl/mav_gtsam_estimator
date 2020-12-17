#include "mav_state_estimation/mav_state_estimator.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_state_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  mav_state_estimation::MavStateEstimator estimator(nh, nh_private);
  ros::spin();
  return 0;
}
