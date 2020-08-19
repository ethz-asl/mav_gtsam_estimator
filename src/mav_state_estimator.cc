#include "mav_state_estimation/mav_state_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

namespace mav_state_estimation {

MavStateEstimator::MavStateEstimator()
    : nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~")) {
  // Get parameters.
  B_t_P_ = getVectorFromParams("position_receiver/B_t");
  ROS_INFO_STREAM("Initial guess B_t_P: " << B_t_P_.transpose());
  B_t_A_ = getVectorFromParams("attitude_receiver/B_t");
  ROS_INFO_STREAM("Initial guess B_t_A: " << B_t_A_.transpose());

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

Eigen::Vector3d
MavStateEstimator::getVectorFromParams(const std::string &param) const {
  std::vector<double> vec;
  nh_private_.getParam(param, vec);
  Eigen::Vector3d eig = Eigen::Vector3d::Zero();
  if (vec.size() == 3) {
    eig = Eigen::Vector3d(vec.data());
  } else {
    ROS_WARN("Cannot process parameter: %s Vector size: %lu", param.c_str(),
             vec.size());
  }
  return eig;
}

void MavStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
  ROS_INFO_ONCE("Received first IMU message.");
  if (!init_g_imu_) {
    // Initialize gravity direction with first measurement.
    // Assumes no motion, no bias.
    init_g_imu_ = std::make_optional<Eigen::Vector3d>();
    tf::vectorMsgToEigen(imu_msg->linear_acceleration, *init_g_imu_);
    *init_g_imu_ *= -1.0;
    ROS_INFO_STREAM("Initialized up direction: " << init_g_imu_->transpose());
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
  if (!heading_init_ && init_g_imu_) {
    Eigen::Vector3d R2;
    tf::pointMsgToEigen(baseline_msg->position.position, R2);
    Eigen::Vector3d r2 = B_t_A_ - B_t_P_;

    // TRIAD method. Correspondences:
    // 1. Gravitation in NED frame R1, and IMU frame r1
    // 2. Moving baseline heading in NED frame R2, and IMU frame r2
    Eigen::Quaterniond q_IB;
    calculateTriadOrientation(Eigen::Vector3d(0, 0, 1), *init_g_imu_, R2, r2,
                              &q_IB);
    heading_init_ = std::make_optional<Eigen::Vector3d>();
  }
}

void MavStateEstimator::calculateTriadOrientation(const Eigen::Vector3d &R1,
                                                  const Eigen::Vector3d &r1,
                                                  const Eigen::Vector3d &R2,
                                                  const Eigen::Vector3d &r2,
                                                  Eigen::Quaterniond *q) const {
  ROS_ASSERT(q);

  // Calculate the rotation between coordinate frames given two linearly
  // independent measurements. https://en.wikipedia.org/wiki/Triad_method
  Eigen::Vector3d S = R1.normalized();
  Eigen::Vector3d s = r1.normalized();
  Eigen::Vector3d M = (R1.cross(R2)).normalized();
  Eigen::Vector3d m = (r1.cross(r2)).normalized();

  Eigen::Matrix3d a;
  a << s, m, s.cross(m);
  ROS_INFO_STREAM("\n" << a);
  Eigen::Matrix3d b;
  b << S, M, S.cross(M);
  ROS_INFO_STREAM("\n" << b);

  Eigen::Matrix3d A = b * a.transpose();
  *q = Eigen::Quaterniond(A);
  ROS_INFO("Computed initial orientation q_IB [x, y, z, w]: [%.2f, %.2f, %.2f, "
           "%.2f]",
           q->x(), q->y(), q->z(), q->w());
}

} // namespace mav_state_estimation

int main(int argc, char **argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::spin();
  return 0;
}
