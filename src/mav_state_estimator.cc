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
  state_.imu_bias =
      gtsam::imuBias::ConstantBias(prior_acc_bias, prior_gyro_bias);
  state_.imu_bias.print("prior_imu_bias: ");

  double bias_acc_sigma = 0.0, bias_omega_sigma = 0.0, bias_acc_int_sigma = 0.0,
         bias_omega_int_sigma = 0.0, acc_sigma = 0.0, integration_sigma = 0.0,
         gyro_sigma = 0.0;
  bool use_2nd_order_coriolis = false;
  nh_private_.getParam("bias_acc_sigma", bias_acc_sigma);
  nh_private_.getParam("bias_omega_sigma", bias_omega_sigma);
  nh_private_.getParam("bias_acc_int_sigma", bias_acc_int_sigma);
  nh_private_.getParam("bias_omega_int_sigma", bias_omega_int_sigma);
  nh_private_.getParam("acc_sigma", acc_sigma);
  nh_private_.getParam("integration_sigma", integration_sigma);
  nh_private_.getParam("gyro_sigma", gyro_sigma);
  nh_private_.getParam("use_2nd_order_coriolis", use_2nd_order_coriolis);

  const gtsam::Matrix I = gtsam::eye(3, 3);
  imu_params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params_->biasAccCovariance = I * pow(bias_acc_sigma, 2);
  imu_params_->biasOmegaCovariance = I * pow(bias_omega_sigma, 2);
  gtsam::Matrix bias_acc_omega_int = gtsam::zeros(6, 6);
  bias_acc_omega_int.block<3, 3>(0, 0) = I * pow(bias_acc_int_sigma, 2);
  bias_acc_omega_int.block<3, 3>(3, 3) = I * pow(bias_omega_int_sigma, 2);
  imu_params_->biasAccOmegaInt = bias_acc_omega_int;
  imu_params_->accelerometerCovariance = I * pow(acc_sigma, 2);
  imu_params_->integrationCovariance = I * pow(integration_sigma, 2);
  imu_params_->gyroscopeCovariance = I * pow(gyro_sigma, 2);
  imu_params_->use2ndOrderCoriolis = use_2nd_order_coriolis;
  imu_params_->print("IMU settings: ");
  imu_integration_ =
      gtsam::PreintegratedCombinedMeasurements(imu_params_, state_.imu_bias);

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
    state_.stamp = T_IB_0.header.stamp;
    state_.nav_state = gtsam::NavState(gtsam::Rot3(q_IB), I_t_B, I_v_B);
    inertial_frame_ = T_IB_0.header.frame_id;
    base_frame_ = T_IB_0.child_frame_id;
    state_.nav_state.print("Initial state: ");
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
  } else if (imu_msg->header.stamp > state_.stamp) {
    // Integrate IMU (zero-order-hold) and publish navigation state.
    //  imu_integration_.resetIntegration();
    Eigen::Vector3d lin_acc, ang_vel;
    tf::vectorMsgToEigen(state_.prev_imu->linear_acceleration, lin_acc);
    tf::vectorMsgToEigen(state_.prev_imu->angular_velocity, ang_vel);
    double dt = (imu_msg->header.stamp - state_.prev_imu->header.stamp).toSec();
    imu_integration_.integrateMeasurement(lin_acc, ang_vel, dt);
    state_.stamp = imu_msg->header.stamp;
    state_.nav_state =
        imu_integration_.predict(state_.nav_state, state_.imu_bias);
    broadcastTf(state_, base_frame_ + "_prediction");
    imu_integration_.resetIntegrationAndSetBias(state_.imu_bias);
    state_.nav_state.print("nav_state:\n");
  }
  state_.prev_imu = imu_msg;
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

void MavStateEstimator::broadcastTf(const State& state,
                                    const std::string& child_frame_id) {
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = state_.stamp;
  tf.header.frame_id = inertial_frame_;
  tf.child_frame_id = child_frame_id;

  tf::vectorEigenToMsg(state.nav_state.position(), tf.transform.translation);
  tf::quaternionEigenToMsg(state.nav_state.attitude().toQuaternion(),
                           tf.transform.rotation);
  tfb_.sendTransform(tf);
}

}  // namespace mav_state_estimation

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::spin();
  return 0;
}
