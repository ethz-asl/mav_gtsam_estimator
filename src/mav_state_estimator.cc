#include "mav_state_estimation/mav_state_estimator.h"

#include <ros/ros.h>

namespace mav_state_estimation {

MavStateEstimator::MavStateEstimator()
    : nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~")) {

  // Subscribe to topics.
  const uint32_t kQueueSize = 1000;
  imu_sub_ =
      nh_.subscribe("imu", kQueueSize, &MavStateEstimator::imuCallback, this);
  pos_0_sub_ =
      nh_.subscribe("pos0", kQueueSize, &MavStateEstimator::posCallback, this);
}

void MavStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
  ROS_INFO_ONCE("Received first IMU message.");
}

void MavStateEstimator::posCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr &pos_msg) {
  ROS_INFO_ONCE("Received first POS message.");
}

} // namespace mav_state_estimation

int main(int argc, char **argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::shutdown();
  return 0;
}
