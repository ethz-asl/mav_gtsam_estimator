#include "mav_state_estimation/mav_state_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <ros/ros.h>

#include "mav_state_estimation/Timing.h"
#include "mav_state_estimation/absolute_position_factor.h"
#include "mav_state_estimation/moving_baseline_factor.h"

using gtsam::symbol_shorthand::A;  // Attitude receiver antenna (x,y,z)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::P;  // Position receiver antenna (x,y,z)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dRow;

namespace mav_state_estimation {

MavStateEstimator::MavStateEstimator()
    : nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~")) {
  // Get parameters.
  // Initial values.
  B_t_P_ = getVectorFromParams("position_receiver/B_t");
  ROS_INFO_STREAM("Initial guess B_t_P: " << B_t_P_.transpose());
  int rate = 0;
  nh_private_.getParam("position_receiver/rate", rate);
  addSensorTimes(rate);
  nh_private_.getParam("position_receiver/scale_cov", pos_receiver_cov_scale_);
  ROS_INFO_STREAM("Position receiver cov scale: " << pos_receiver_cov_scale_);
  double prior_noise_B_t_P;
  nh_private_.getParam("position_receiver/prior_noise_B_t", prior_noise_B_t_P);
  auto prior_noise_model_B_t_P =
      gtsam::noiseModel::Isotropic::Sigma(B_t_P_.size(), prior_noise_B_t_P);
  auto prior_B_t_P = boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
      P(0), B_t_P_, prior_noise_model_B_t_P);
  prior_B_t_P->print("Prior noise model B_t_P:\n");
  new_unary_factors_.emplace_back(0, prior_B_t_P);
  double process_noise_B_t_P;
  nh_private_.getParam("position_receiver/process_noise_B_t",
                       process_noise_B_t_P);
  process_noise_model_B_t_P_ =
      gtsam::noiseModel::Isotropic::Sigma(B_t_P_.size(), process_noise_B_t_P);
  process_noise_model_B_t_P_->print("Process noise B_t_P:\n");

  B_t_A_ = getVectorFromParams("attitude_receiver/B_t");
  ROS_INFO_STREAM("Initial guess B_t_A: " << B_t_A_.transpose());
  nh_private_.getParam("attitude_receiver/rate", rate);
  addSensorTimes(rate);
  nh_private_.getParam("attitude_receiver/scale_cov", att_receiver_cov_scale_);
  ROS_INFO_STREAM("Attitude receiver cov scale: " << att_receiver_cov_scale_);
  double prior_noise_B_t_A;
  nh_private_.getParam("attitude_receiver/prior_noise_B_t", prior_noise_B_t_A);
  auto prior_noise_model_B_t_A =
      gtsam::noiseModel::Isotropic::Sigma(B_t_A_.size(), prior_noise_B_t_A);
  auto prior_B_t_A = boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
      A(0), B_t_A_, prior_noise_model_B_t_A);
  prior_B_t_A->print("Prior noise model B_t_A:\n");
  new_unary_factors_.emplace_back(0, prior_B_t_A);
  double process_noise_B_t_A;
  nh_private_.getParam("attitude_receiver/process_noise_B_t",
                       process_noise_B_t_A);
  process_noise_model_B_t_A_ =
      gtsam::noiseModel::Isotropic::Sigma(B_t_A_.size(), process_noise_B_t_A);
  process_noise_model_B_t_A_->print("Process noise B_t_A:\n");

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
  prev_bias_ = gtsam::imuBias::ConstantBias(prior_acc_bias, prior_gyro_bias);
  prev_bias_.print("prior_imu_bias: ");

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
  auto imu_params =
      gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params->biasAccCovariance = I * pow(bias_acc_sigma, 2);
  imu_params->biasOmegaCovariance = I * pow(bias_omega_sigma, 2);
  gtsam::Matrix bias_acc_omega_int = gtsam::zeros(6, 6);
  bias_acc_omega_int.block<3, 3>(0, 0) = I * pow(bias_acc_int_sigma, 2);
  bias_acc_omega_int.block<3, 3>(3, 3) = I * pow(bias_omega_int_sigma, 2);
  imu_params->biasAccOmegaInt = bias_acc_omega_int;
  imu_params->accelerometerCovariance = I * pow(acc_sigma, 2);
  imu_params->integrationCovariance = I * pow(integration_sigma, 2);
  imu_params->gyroscopeCovariance = I * pow(gyro_sigma, 2);
  imu_params->use2ndOrderCoriolis = use_2nd_order_coriolis;
  imu_params->print("IMU settings: ");
  integrator_ =
      gtsam::PreintegratedCombinedMeasurements(imu_params, prev_bias_);

  gtsam::ISAM2Params parameters;
  // TODO(rikba): Set more ISAM2 params here.
  double relinearize_threshold_rot, relinearize_threshold_pos,
      relinearize_threshold_vel, relinearize_threshold_acc_bias,
      relinearize_threshold_gyro_bias,
      relinearize_threshold_antenna_calibration;
  nh_private_.getParam("isam2/relinearize_threshold_rot",
                       relinearize_threshold_rot);
  nh_private_.getParam("isam2/relinearize_threshold_pos",
                       relinearize_threshold_pos);
  nh_private_.getParam("isam2/relinearize_threshold_vel",
                       relinearize_threshold_vel);
  nh_private_.getParam("isam2/relinearize_threshold_acc_bias",
                       relinearize_threshold_acc_bias);
  nh_private_.getParam("isam2/relinearize_threshold_gyro_bias",
                       relinearize_threshold_gyro_bias);
  nh_private_.getParam("isam2/relinearize_threshold_antenna_calibration",
                       relinearize_threshold_antenna_calibration);
  gtsam::FastMap<char, gtsam::Vector> thresholds;
  thresholds['x'] =
      (gtsam::Vector(6) << Eigen::Vector3d::Constant(relinearize_threshold_rot),
       Eigen::Vector3d::Constant(relinearize_threshold_pos))
          .finished();
  thresholds['v'] = Eigen::Vector3d::Constant(relinearize_threshold_vel);
  thresholds['b'] = (gtsam::Vector(6) << Eigen::Vector3d::Constant(
                         relinearize_threshold_acc_bias),
                     Eigen::Vector3d::Constant(relinearize_threshold_gyro_bias))
                        .finished();
  thresholds['p'] =
      Eigen::Vector3d::Constant(relinearize_threshold_antenna_calibration);
  thresholds['a'] =
      Eigen::Vector3d::Constant(relinearize_threshold_antenna_calibration);
  parameters.relinearizeThreshold = thresholds;
  // parameters.optimizationParams = gtsam::ISAM2DoglegParams();
  nh_private_.getParam("isam2/relinearize_skip", parameters.relinearizeSkip);
  isam2_ = gtsam::ISAM2(parameters);

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

  // Advertise topics.
  timing_pub_ = nh_private_.advertise<mav_state_estimation::Timing>(
      "solveThreaded", kQueueSize);
  prediction_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>(
      "prediction", kQueueSize);
  optimization_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>(
      "optimization", kQueueSize);
  acc_bias_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "acc_bias", kQueueSize);
  gyro_bias_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "gyro_bias", kQueueSize);
  position_antenna_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "position_antenna", kQueueSize);
  attitude_antenna_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "attitude_antenna", kQueueSize);
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

void MavStateEstimator::addSensorTimes(const uint16_t rate) {
  uint32_t period = 1e9 / rate;
  uint32_t ns = 0;
  while (ns < 1e9) {
    auto success = unary_times_ns_.insert(ns);
    if (success.second) {
      ROS_INFO("Inserted new sensor time: %u", ns);
    }
    ns += period;
  }
}

void MavStateEstimator::initializeState() {
  geometry_msgs::TransformStamped T_IB_0;
  if (init_.getInitialPose(&T_IB_0)) {
    // Get initial values.
    Eigen::Vector3d I_t_B;
    Eigen::Quaterniond q_IB;
    tf::vectorMsgToEigen(T_IB_0.transform.translation, I_t_B);
    tf::quaternionMsgToEigen(T_IB_0.transform.rotation, q_IB);
    Eigen::Vector3d I_v_B = Eigen::Vector3d::Zero();
    gtsam::Pose3 T_IB(gtsam::Rot3(q_IB), I_t_B);

    // Fill initial ISAM state.
    new_values_.insert(X(0), T_IB);
    new_values_.insert(V(0), I_v_B);
    new_values_.insert(B(0), prev_bias_);
    new_values_.insert(P(0), B_t_P_);
    new_values_.insert(A(0), B_t_A_);

    prev_state_ = gtsam::NavState(T_IB, I_v_B);

    auto prior_pose = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(0), T_IB, prior_noise_model_T_I_B_);
    auto prior_vel = boost::make_shared<gtsam::PriorFactor<gtsam::Velocity3>>(
        V(0), I_v_B, prior_noise_model_I_v_B_);
    auto prior_bias =
        boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
            B(0), prev_bias_, prior_noise_model_imu_bias_);
    new_unary_factors_.emplace_back(0, prior_pose);
    new_unary_factors_.emplace_back(0, prior_vel);
    new_unary_factors_.emplace_back(0, prior_bias);

    // Initialize time stamps.
    stamp_to_idx_[T_IB_0.header.stamp] = 0;
    idx_to_stamp_[stamp_to_idx_[T_IB_0.header.stamp]] = T_IB_0.header.stamp;
    auto next_state_time = T_IB_0.header.stamp;
    next_state_time.nsec =
        *unary_times_ns_.upper_bound(T_IB_0.header.stamp.nsec);
    addUnaryStamp(next_state_time);

    // Print
    new_values_.print("Initial state: ");
    ROS_INFO_STREAM("Initialization stamp: " << idx_to_stamp_[idx_]);
    ROS_INFO_STREAM("Next unary stamp: " << idx_to_stamp_[1]);
  }
}

void MavStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  // Do not update prediction while initial values are updated.
  std::unique_lock<std::recursive_mutex> lock(update_mtx_);
  ROS_INFO_ONCE("Received first IMU message.");
  if (!isInitialized()) {
    // Gravitational acceleration in inertial frame (ENU).
    Eigen::Vector3d I_g(0, 0, -9.81);
    // Gravitational acceleration in base frame (IMU).
    Eigen::Vector3d B_g;
    tf::vectorMsgToEigen(imu_msg->linear_acceleration, B_g);
    B_g *= -1.0;
    base_frame_ = imu_msg->header.frame_id;
    init_.setBaseFrame(base_frame_);
    init_.addOrientationConstraint1(I_g, B_g, imu_msg->header.stamp);
    initializeState();
  } else if (imu_msg->header.stamp > idx_to_stamp_[idx_ + 1]) {
    // Handle dropped IMU message.
    // TODO(rikba): Linearly inpterpolate IMU message.
    auto in_between_imu = boost::make_shared<sensor_msgs::Imu>(*prev_imu_);
    in_between_imu->header.stamp = idx_to_stamp_[idx_ + 1];
    ROS_WARN_STREAM("Inserting missing IMU message at "
                    << in_between_imu->header.stamp);
    ROS_WARN_STREAM("Prev IMU stamp: " << prev_imu_->header.stamp);
    ROS_WARN_STREAM("This IMU stamp: " << imu_msg->header.stamp);
    imuCallback(in_between_imu);
  } else if (imu_msg->header.stamp > prev_imu_->header.stamp) {
    // Integrate IMU (zero-order-hold).
    Eigen::Vector3d lin_acc, ang_vel;
    tf::vectorMsgToEigen(prev_imu_->linear_acceleration, lin_acc);
    tf::vectorMsgToEigen(prev_imu_->angular_velocity, ang_vel);
    double dt = (imu_msg->header.stamp - prev_imu_->header.stamp).toSec();
    integrator_.integrateMeasurement(lin_acc, ang_vel, dt);

    // Publish high rate IMU prediction.
    auto imu_state = integrator_.predict(prev_state_, prev_bias_);
    broadcastTf(imu_state, imu_msg->header.stamp, base_frame_ + "_prediction");
    publishPose(imu_state, imu_msg->header.stamp, prediction_pub_);

    // Setup new inbetween IMU factor.
    if (addUnaryStamp(imu_msg->header.stamp)) {
      idx_ = stamp_to_idx_[imu_msg->header.stamp];
      prev_state_ = imu_state;

      new_values_.insert(B(idx_), prev_bias_);
      new_values_.insert(X(idx_), prev_state_.pose());
      new_values_.insert(V(idx_), prev_state_.v());

      auto imu_factor = boost::make_shared<gtsam::CombinedImuFactor>(
          X(idx_ - 1), V(idx_ - 1), X(idx_), V(idx_), B(idx_ - 1), B(idx_),
          integrator_);
      integrator_.resetIntegrationAndSetBias(prev_bias_);
      new_graph_.add(imu_factor);

      // Add antenna offset calibration factors.
      new_graph_.add(gtsam::BetweenFactor<gtsam::Point3>(
          P(idx_ - 1), P(idx_), gtsam::Point3::Zero(),
          process_noise_model_B_t_P_));
      new_graph_.add(gtsam::BetweenFactor<gtsam::Point3>(
          A(idx_ - 1), A(idx_), gtsam::Point3::Zero(),
          process_noise_model_B_t_A_));

      new_values_.insert(P(idx_), B_t_P_);
      new_values_.insert(A(idx_), B_t_A_);

      // Attempt to run solver thread.
      solve();
    }
  } else {
    ROS_ERROR("Cannot handle IMU message.");
  }
  prev_imu_ = imu_msg;
}

void MavStateEstimator::posCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr& pos_msg) {
  // Do not update factors while initial values are updated.
  std::unique_lock<std::recursive_mutex> lock(update_mtx_);
  ROS_INFO_ONCE("Received first POS message.");
  Eigen::Vector3d I_t_P;
  tf::pointMsgToEigen(pos_msg->position.position, I_t_P);
  if (!isInitialized()) {
    // GNSS antenna position in inertial frame (ENU).
    inertial_frame_ = pos_msg->header.frame_id;
    init_.setInertialFrame(inertial_frame_);
    init_.addPositionConstraint(I_t_P, B_t_P_, pos_msg->header.stamp);
  } else if (addUnaryStamp(pos_msg->header.stamp)) {
    const bool kSmart = false;
    auto cov = gtsam::noiseModel::Gaussian::Covariance(
        pos_receiver_cov_scale_ *
            Matrix3dRow::Map(pos_msg->position.covariance.data()),
        kSmart);
    AbsolutePositionFactor::shared_ptr pos_factor =
        boost::make_shared<AbsolutePositionFactor>(
            X(stamp_to_idx_[pos_msg->header.stamp]),
            P(stamp_to_idx_[pos_msg->header.stamp]), I_t_P, cov);
    new_unary_factors_.emplace_back(stamp_to_idx_[pos_msg->header.stamp],
                                    pos_factor);
    solve();
  } else {
    ROS_WARN("Failed to add unary position factor.");
  }
}

bool MavStateEstimator::addUnaryStamp(const ros::Time& stamp) {
  bool valid = (stamp >= stamp_to_idx_.begin()->first);
  ROS_WARN_COND(!valid,
                "The new stamp %u.%u is before initialization time %u.%u.",
                stamp.sec, stamp.nsec, stamp_to_idx_.begin()->first.sec,
                stamp_to_idx_.begin()->first.nsec);
  bool is_unary = valid & unary_times_ns_.count(stamp.nsec);
  ROS_DEBUG_COND(!is_unary,
                 "The new stamp %u.%u is not expected as a unary factor time.",
                 stamp.sec, stamp.nsec);

  // Always make sure that the previous, current, and next n seconds are part
  // of the index lookup map already.
  const uint8_t kIdxWindow = 1;
  auto max_time = ros::Time(stamp.sec + kIdxWindow + 1, 0);
  if (!stamp_to_idx_.count(max_time)) {
    auto min_time = std::max(stamp_to_idx_.rbegin()->first,
                             ros::Time(stamp.sec - kIdxWindow, 0));
    auto ns = unary_times_ns_.begin();
    while (!stamp_to_idx_.count(max_time)) {
      if (!stamp_to_idx_.count(min_time) &&
          min_time > stamp_to_idx_.rbegin()->first) {
        stamp_to_idx_[min_time] = stamp_to_idx_.rbegin()->second + 1;
        idx_to_stamp_[stamp_to_idx_[min_time]] = min_time;
      }
      ns = std::next(ns);
      if (ns == unary_times_ns_.end()) {
        min_time.sec += 1;
        ns = unary_times_ns_.begin();
      }
      min_time.nsec = *ns;
    }
  }

  return is_unary;
}

void MavStateEstimator::baselineCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr&
        baseline_msg) {
  // Do not update factors while initial values are updated.
  std::unique_lock<std::recursive_mutex> lock(update_mtx_);
  ROS_INFO_ONCE("Received first BASELINE message.");
  Eigen::Vector3d NED_t_PA;
  tf::pointMsgToEigen(baseline_msg->position.position, NED_t_PA);
  // TODO(rikba): Use ECEF frame by default.
  // TODO(rikba): Account for different frame positions. This rotation is only
  // correct if ENU and NED origin coincide.
  const auto R_ENU_NED =
      (Eigen::Matrix3d() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
          .finished();
  Eigen::Vector3d I_t_PA = R_ENU_NED * NED_t_PA;
  // Moving baseline heading in base frame (IMU).
  const Eigen::Vector3d B_t_PA = B_t_A_ - B_t_P_;
  if (!isInitialized()) {
    init_.addOrientationConstraint2(I_t_PA, B_t_PA, baseline_msg->header.stamp);
  } else if (addUnaryStamp(baseline_msg->header.stamp)) {
    auto cov = gtsam::noiseModel::Gaussian::Covariance(
        R_ENU_NED * att_receiver_cov_scale_ *
        Matrix3dRow::Map(baseline_msg->position.covariance.data()) *
        R_ENU_NED.transpose());
    MovingBaselineFactor::shared_ptr baseline_factor =
        boost::make_shared<MovingBaselineFactor>(
            X(stamp_to_idx_[baseline_msg->header.stamp]),
            P(stamp_to_idx_[baseline_msg->header.stamp]),
            A(stamp_to_idx_[baseline_msg->header.stamp]), I_t_PA, cov);
    new_unary_factors_.emplace_back(stamp_to_idx_[baseline_msg->header.stamp],
                                    baseline_factor);
    solve();
  } else {
    ROS_WARN("Failed to add unary baseline factor.");
  }
}

void MavStateEstimator::broadcastTf(const gtsam::NavState& state,
                                    const ros::Time& stamp,
                                    const std::string& child_frame_id) {
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = inertial_frame_;
  tf.child_frame_id = child_frame_id;

  tf::vectorEigenToMsg(state.position(), tf.transform.translation);
  tf::quaternionEigenToMsg(state.attitude().toQuaternion(),
                           tf.transform.rotation);
  tfb_.sendTransform(tf);
}

void MavStateEstimator::publishPose(const gtsam::NavState& state,
                                    const ros::Time& stamp,
                                    const ros::Publisher& pub) const {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = inertial_frame_;

  tf::pointEigenToMsg(state.position(), pose.pose.position);
  tf::quaternionEigenToMsg(state.attitude().toQuaternion(),
                           pose.pose.orientation);
  pub.publish(pose);
}

void MavStateEstimator::publishAntennaPosition(
    const gtsam::Point3& B_t, const ros::Time& stamp,
    const ros::Publisher& pub) const {
  geometry_msgs::Vector3Stamped antenna_position;
  antenna_position.header.stamp = stamp;
  antenna_position.header.frame_id = base_frame_;
  tf::vectorEigenToMsg(B_t, antenna_position.vector);
  pub.publish(antenna_position);
}

void MavStateEstimator::publishBias(const gtsam::imuBias::ConstantBias& bias,
                                    const ros::Time& stamp) const {
  geometry_msgs::Vector3Stamped acc_bias, gyro_bias;
  acc_bias.header.stamp = stamp;
  gyro_bias.header.stamp = stamp;

  tf::vectorEigenToMsg(bias.accelerometer(), acc_bias.vector);
  tf::vectorEigenToMsg(bias.gyroscope(), gyro_bias.vector);

  acc_bias_pub_.publish(acc_bias);
  gyro_bias_pub_.publish(gyro_bias);
}

MavStateEstimator::~MavStateEstimator() {
  if (solver_thread_.joinable()) solver_thread_.join();
}

void MavStateEstimator::solve() {
  // Transfer new unary factors to graph if IMU inbetween exists.
  new_unary_factors_.erase(
      std::remove_if(
          new_unary_factors_.begin(), new_unary_factors_.end(),
          [this](const std::pair<uint64_t, gtsam::NonlinearFactor::shared_ptr>&
                     factor) -> bool {
            bool remove = (factor.first <= this->idx_);
            if (remove) {
              this->new_graph_.add(factor.second);
            }
            return remove;
          }),
      new_unary_factors_.end());

  // Check for new result.
  if (is_solving_.load() || new_graph_.empty()) {
    return;
  }
  if (solver_thread_.joinable()) solver_thread_.join();

  // Copy new factors to graph.
  auto graph = std::make_unique<gtsam::NonlinearFactorGraph>(new_graph_);
  auto values = std::make_unique<gtsam::Values>(new_values_);
  auto time = std::make_unique<ros::Time>(idx_to_stamp_[idx_]);
  new_graph_.resize(0);
  new_values_.clear();

  // Solve.
  is_solving_.store(true);
  solver_thread_ =
      std::thread(&MavStateEstimator::solveThreaded, this, std::move(graph),
                  std::move(values), std::move(time), idx_);
}

void MavStateEstimator::solveThreaded(
    std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
    std::unique_ptr<gtsam::Values> values, std::unique_ptr<ros::Time> time,
    uint64_t i) {
  assert(graph);
  assert(values);
  assert(time);

  // Solve iterative problem.
  gttic_(solveThreaded);
  isam2_.update(*graph, *values);
  auto pose = isam2_.calculateEstimate<gtsam::Pose3>(X(i));
  auto velocity = isam2_.calculateEstimate<gtsam::Velocity3>(V(i));
  auto bias = isam2_.calculateEstimate<gtsam::imuBias::ConstantBias>(B(i));
  auto B_t_P = isam2_.calculateEstimate<gtsam::Point3>(P(i));
  auto B_t_A = isam2_.calculateEstimate<gtsam::Point3>(A(i));
  gttoc_(solveThreaded);
  gtsam::tictoc_finishedIteration_();

  // Print.
  // static uint32_t iteration = 0;
  // char buffer[50];
  // sprintf(buffer, "/tmp/graph_%04d.dot", iteration);
  // std::ofstream os(buffer);
  // ROS_INFO_STREAM("Storing graph " << iteration);
  // isam2_.getFactorsUnsafe().saveGraph(os, isam2_.getLinearizationPoint());
  // ROS_INFO_STREAM("Storing bayes " << iteration);
  // sprintf(buffer, "/tmp/bayes_%04d.dot", iteration++);
  // isam2_.saveGraph(buffer);

  // ROS publishers
  tictoc_getNode(solveThreaded, solveThreaded);
  timing_msg_.header.stamp = *time;
  timing_msg_.iteration = solveThreaded->self() - timing_msg_.time;
  timing_msg_.time = solveThreaded->self();
  timing_msg_.min = solveThreaded->min();
  timing_msg_.max = solveThreaded->max();
  timing_msg_.mean = solveThreaded->mean();
  timing_pub_.publish(timing_msg_);

  gtsam::NavState new_state(pose, velocity);
  broadcastTf(new_state, *time, base_frame_ + "_optimization");
  publishPose(new_state, *time, optimization_pub_);
  publishBias(bias, *time);
  publishAntennaPosition(B_t_P, *time, position_antenna_pub_);
  publishAntennaPosition(B_t_A, *time, attitude_antenna_pub_);

  // Update new values (threadsafe, blocks all sensor callbacks).
  std::unique_lock<std::recursive_mutex> lock(update_mtx_);
  prev_state_ = new_state;
  prev_bias_ = bias;
  B_t_P_ = B_t_P;
  B_t_A_ = B_t_A;

  for (auto it = new_graph_.begin(); it != new_graph_.end(); ++it) {
    auto imu = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(*it);
    if (imu) {
      prev_state_ =
          imu->preintegratedMeasurements().predict(prev_state_, prev_bias_);
      new_values_.update(X(i + 1), prev_state_.pose());
      new_values_.update(V(i + 1), prev_state_.velocity());
      new_values_.update(B(i + 1), prev_bias_);
      new_values_.update(P(i + 1), B_t_P_);
      new_values_.update(A(i + 1), B_t_A_);
      i++;
    }
  }
  assert(idx_ == i);

  is_solving_.store(false);
}

}  // namespace mav_state_estimation

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::spin();
  return 0;
}
