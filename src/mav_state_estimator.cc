#include "mav_state_estimation/mav_state_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

namespace mav_state_estimation {

MavStateEstimator::MavStateEstimator()
    : nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~")) {
  // Get parameters.
  // Initial values.
  B_t_P_ = getVectorFromParams("position_receiver/B_t");
  ROS_INFO_STREAM("Initial guess B_t_P: " << B_t_P_.transpose());
  B_t_A_ = getVectorFromParams("attitude_receiver/B_t");
  ROS_INFO_STREAM("Initial guess B_t_A: " << B_t_A_.transpose());

  Eigen::Vector3d prior_noise_rot_IB, prior_noise_I_t_B;
  prior_noise_rot_IB = getVectorFromParams("prior_noise_rot_IB");
  prior_noise_I_t_B = getVectorFromParams("prior_noise_I_t_B");
  prior_noise_model_T_I_B_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << prior_noise_rot_IB, prior_noise_I_t_B).finished());
  prior_noise_model_T_I_B_->print("prior_noise_model_T_I_B: ");

  Eigen::Vector3d prior_noise_I_v_B;
  prior_noise_I_v_B = getVectorFromParams("prior_noise_I_v_B");
  prior_noise_model_I_v_B_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << prior_noise_I_v_B).finished());
  prior_noise_model_I_v_B_->print("prior_noise_model_I_v_B: ");

  Eigen::Vector3d prior_noise_acc_bias, prior_noise_gyro_bias;
  prior_noise_acc_bias = getVectorFromParams("prior_noise_acc_bias");
  prior_noise_gyro_bias = getVectorFromParams("prior_noise_gyro_bias");
  prior_noise_model_imu_bias_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << prior_noise_acc_bias, prior_noise_gyro_bias)
          .finished());
  prior_noise_model_imu_bias_->print("prior_noise_model_imu_bias: ");

  Eigen::Vector3d prior_acc_bias, prior_gyro_bias;
  prior_acc_bias = getVectorFromParams("prior_acc_bias");
  prior_gyro_bias = getVectorFromParams("prior_gyro_bias");
  prior_imu_bias_ =
      gtsam::imuBias::ConstantBias(prior_acc_bias, prior_gyro_bias);
  prior_imu_bias_.print("prior_imu_bias: ");

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

Eigen::Vector3d MavStateEstimator::getVectorFromParams(
    const std::string& param) const {
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

void MavStateEstimator::initializeGraph() {
  geometry_msgs::TransformStamped T_IB_0;
  if (init_.getInitialPose(&T_IB_0)) {
    Eigen::Vector3d I_t_B;
    Eigen::Quaterniond q_IB;
    tf::vectorMsgToEigen(T_IB_0.transform.translation, I_t_B);
    tf::quaternionMsgToEigen(T_IB_0.transform.rotation, q_IB);

    Eigen::Vector3d I_v_B = Eigen::Vector3d::Zero();
    nav_state_ = gtsam::NavState(gtsam::Rot3(q_IB), I_t_B, I_v_B);
    nav_state_.print("Initial state: ");
  }
}

void MavStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  ROS_INFO_ONCE("Received first IMU message.");
  if (!init_.isInitialized()) {
    // Gravitational acceleration in inertial frame (ENU).
    Eigen::Vector3d I_g(0, 0, -9.81);
    // Gravitational acceleration in base frame (IMU).
    Eigen::Vector3d B_g;
    tf::vectorMsgToEigen(imu_msg->linear_acceleration, B_g);
    B_g *= -1.0;
    init_.addOrientationConstraint1(I_g, B_g, imu_msg->header.stamp);
    init_.setBaseFrame(imu_msg->header.frame_id);
    initializeGraph();
  } else {
    // Integrate IMU and publish navigation state.
  }
}

void MavStateEstimator::posCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr& pos_msg) {
  ROS_INFO_ONCE("Received first POS message.");
  if (!init_.isInitialized()) {
    // GNSS antenna position in inertial frame (ENU).
    Eigen::Vector3d I_t_P;
    tf::pointMsgToEigen(pos_msg->position.position, I_t_P);
    init_.addPositionConstraint(I_t_P, B_t_P_, pos_msg->header.stamp);
    init_.setInertialFrame(pos_msg->header.frame_id);
    initializeGraph();
  }
}

void MavStateEstimator::baselineCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr&
        baseline_msg) {
  ROS_INFO_ONCE("Received first BASELINE message.");
  if (!init_.isInitialized()) {
    // TODO(rikba): Use ECEF frame by default.
    // Moving baseline heading in inertial frame (ENU).
    Eigen::Vector3d I_h_NED;
    tf::pointMsgToEigen(baseline_msg->position.position, I_h_NED);
    Eigen::Vector3d I_h(I_h_NED.y(), I_h_NED.x(), -I_h_NED.z());
    // Moving baseline heading in base frame (IMU).
    Eigen::Vector3d B_h = B_t_A_ - B_t_P_;
    init_.addOrientationConstraint2(I_h, B_h, baseline_msg->header.stamp);
    initializeGraph();
  }
}

}  // namespace mav_state_estimation

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::spin();
  return 0;
}
