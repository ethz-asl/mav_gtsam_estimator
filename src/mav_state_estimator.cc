#include "mav_state_estimation/mav_state_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>

namespace mav_state_estimation {

MavStateEstimator::MavStateEstimator()
    : nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~")) {

  // Subscribe to topics.
  const uint32_t kQueueSize = 1000;
  imu_sub_ =
      nh_.subscribe("imu0", kQueueSize, &MavStateEstimator::imuCallback, this);
  ROS_INFO("Subscribing to: %s", imu_sub_.getTopic().c_str());
  pos_0_sub_ =
      nh_.subscribe("pos0", kQueueSize, &MavStateEstimator::posCallback, this);
  ROS_INFO("Subscribing to: %s", pos_0_sub_.getTopic().c_str());
  baseline_sub_ = nh_.subscribe("baseline0", kQueueSize,
                                &MavStateEstimator::baselineCallback, this);
  ROS_INFO("Subscribing to: %s", baseline_sub_.getTopic().c_str());
}

void MavStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
  ROS_INFO_ONCE("Received first IMU message.");
  if (up_init_) {

  } else {
    // Initialize gravity direction with first measurement.
    // Assumes no motion, no bias.
    up_init_ = std::make_optional<Eigen::Vector3d>();
    tf::vectorMsgToEigen(imu_msg->linear_acceleration, *up_init_);
    up_init_->normalize();
    ROS_INFO_STREAM("Initialized up direction: " << up_init_->transpose());
  }
}

void MavStateEstimator::posCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr &pos_msg) {
  ROS_INFO_ONCE("Received first POS message.");
}

void MavStateEstimator::baselineCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr
        &baseline_msg) {
  ROS_INFO_ONCE("Received first BASELINE message.");
}

} // namespace mav_state_estimation

int main(int argc, char **argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::spin();
  return 0;
}
