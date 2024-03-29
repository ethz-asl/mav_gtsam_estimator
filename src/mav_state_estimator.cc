/*
MIT License
Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "mav_state_estimation/mav_state_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>

#include "mav_state_estimation/BatchStatus.h"
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

MavStateEstimator::MavStateEstimator(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Get parameters.
  nh_private_.getParam("external_poses", external_poses_);
  nh_private_.getParam("odometry_throttle", odometry_throttle_);

  // GNSS
  nh_private_.getParam("estimate_antenna_positions",
                       estimate_antenna_positions_);
  loadGnssParams("position_receiver", P(0), &B_t_P_,
                 &process_noise_model_B_t_P_, &pos_receiver_cov_scale_);
  loadGnssParams("attitude_receiver", A(0), &B_t_A_,
                 &process_noise_model_B_t_A_, &att_receiver_cov_scale_);

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

  // ISAM2
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
  nh_private_.getParam("isam2/relinearize_skip", parameters.relinearizeSkip);
  nh_private_.getParam("isam2/enable_partial_relinarization_check",
                       parameters.enablePartialRelinearizationCheck);

  double smoother_lag = 0.0;
  nh_private_.getParam("isam2/smoother_lag", smoother_lag);

  smoother_ = gtsam::IncrementalFixedLagSmoother(smoother_lag, parameters);

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
  prediction_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "prediction", kQueueSize);
  optimization_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "optimization", kQueueSize);
  acc_bias_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "acc_bias", kQueueSize);
  gyro_bias_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "gyro_bias", kQueueSize);
  position_antenna_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "position_antenna", kQueueSize);
  attitude_antenna_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
      "attitude_antenna", kQueueSize);
  odometry_pub_ =
      nh_private_.advertise<nav_msgs::Odometry>("odometry", kQueueSize);

  batch_srv_ = nh_private_.advertiseService(
      "compute_batch_solution", &MavStateEstimator::computeBatchSolution, this);
}

void MavStateEstimator::loadGnssParams(
    const std::string& antenna_ns, const gtsam::Symbol& symbol,
    gtsam::Point3* B_t, gtsam::noiseModel::Isotropic::shared_ptr* process_noise,
    double* cov_scale) {
  assert(B_t);
  assert(cov_scale);

  // General GNSS parameters.
  *B_t = getVectorFromParams(antenna_ns + "/B_t");
  ROS_INFO_STREAM("Initial guess " << antenna_ns.c_str()
                                   << " location: " << B_t->transpose());
  int rate = 0;
  nh_private_.getParam(antenna_ns + "/rate", rate);
  ROS_INFO_STREAM(antenna_ns.c_str() << " rate: " << rate << " Hz");
  addSensorTimes(rate);
  nh_private_.getParam(antenna_ns + "/scale_cov", *cov_scale);
  ROS_INFO_STREAM(antenna_ns.c_str() << " cov scale: " << *cov_scale);

  if (estimate_antenna_positions_) {
    // Prior
    double prior_noise_B_t;
    nh_private_.getParam(antenna_ns + "/prior_noise_B_t", prior_noise_B_t);
    auto prior_noise =
        gtsam::noiseModel::Isotropic::Sigma(B_t->size(), prior_noise_B_t);
    auto prior_B_t = boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
        symbol, *B_t, prior_noise);
    prior_B_t->print(antenna_ns + " prior factor:\n");
    new_unary_factors_.emplace_back(0, prior_B_t);

    // Initial value
    new_values_.insert(symbol, *B_t);

    // Process noise
    double process_noise_B_t;
    nh_private_.getParam(antenna_ns + "/process_noise_B_t", process_noise_B_t);
    *process_noise =
        gtsam::noiseModel::Isotropic::Sigma(B_t->size(), process_noise_B_t);
    process_noise_model_B_t_P_->print(antenna_ns + " process noise:\n");
  }
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
    updateTimestampMap();

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
    Eigen::Vector3d I_g = integrator_.p().getGravity();
    // Gravitational acceleration in base frame (IMU).
    Eigen::Vector3d B_g;
    tf::vectorMsgToEigen(imu_msg->linear_acceleration, B_g);
    B_g = prev_bias_.correctAccelerometer(B_g);
    B_g *= -1.0;
    base_frame_ = imu_msg->header.frame_id;
    ROS_INFO_ONCE("Base frame: %s", base_frame_.c_str());
    init_.setBaseFrame(base_frame_);
    init_.addOrientationConstraint1(I_g, B_g, imu_msg->header.stamp);
    initializeState();
  } else if (imu_msg->header.stamp > idx_to_stamp_[idx_ + 1]) {
    // Handle dropped IMU message.
    // TODO(rikba): Linearly inpterpolate IMU message.
    auto in_between_imu = boost::make_shared<sensor_msgs::Imu>(*prev_imu_);
    in_between_imu->header.stamp = idx_to_stamp_[idx_ + 1];
    ROS_DEBUG_STREAM("Inserting missing IMU message at "
                     << in_between_imu->header.stamp);
    ROS_DEBUG_STREAM("Prev IMU stamp: " << prev_imu_->header.stamp);
    ROS_DEBUG_STREAM("This IMU stamp: " << imu_msg->header.stamp);
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
    if ((imu_msg->header.seq % odometry_throttle_) == 0) {
      geometry_msgs::TransformStamped T_IB =
          getTransform(imu_state, imu_msg->header.stamp, base_frame_);
      tfb_.sendTransform(T_IB);
      prediction_pub_.publish(T_IB);
      publishOdometry(imu_state.velocity(), ang_vel, prev_bias_,
                      imu_msg->header.stamp, T_IB);
    }

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

      if (estimate_antenna_positions_) {
        // Add antenna offset calibration factors.
        new_graph_.add(gtsam::BetweenFactor<gtsam::Point3>(
            P(idx_ - 1), P(idx_), gtsam::Point3::Zero(),
            process_noise_model_B_t_P_));
        new_graph_.add(gtsam::BetweenFactor<gtsam::Point3>(
            A(idx_ - 1), A(idx_), gtsam::Point3::Zero(),
            process_noise_model_B_t_A_));

        new_values_.insert(P(idx_), B_t_P_);
        new_values_.insert(A(idx_), B_t_A_);
      }

      updateTimestampMap();

      // Attempt to run solver thread.
      solve();
    }
  } else {
    ROS_ERROR("Cannot handle IMU message.");
    ROS_ERROR_STREAM("This IMU stamp: " << imu_msg->header.stamp);
    ROS_ERROR_STREAM("Prev IMU stamp: " << prev_imu_->header.stamp);
  }
  prev_imu_ = imu_msg;
  batch_imu_.push_back(imu_msg);
}

void MavStateEstimator::updateTimestampMap() {
  auto new_stamp = idx_to_stamp_[idx_].toSec();
  new_timestamps_[B(idx_)] = new_stamp;
  new_timestamps_[X(idx_)] = new_stamp;
  new_timestamps_[V(idx_)] = new_stamp;
  if (estimate_antenna_positions_) {
    new_timestamps_[P(idx_)] = new_stamp;
    new_timestamps_[A(idx_)] = new_stamp;
  }
}

void MavStateEstimator::posCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr& pos_msg) {
  static piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr prev_msg =
      nullptr;
  // Do not update factors while initial values are updated.
  std::unique_lock<std::recursive_mutex> lock(update_mtx_);
  ROS_INFO_ONCE("Received first POS message.");
  Eigen::Vector3d I_t_P;
  tf::pointMsgToEigen(pos_msg->position.position, I_t_P);
  if (!isInitialized()) {
    // GNSS antenna position in inertial frame (ENU).
    inertial_frame_ = pos_msg->header.frame_id;
    ROS_INFO_ONCE("Inertial frame: %s", inertial_frame_.c_str());
    init_.setInertialFrame(inertial_frame_);
    init_.addPositionConstraint(I_t_P, B_t_P_, pos_msg->header.stamp);
  } else if (addUnaryStamp(pos_msg->header.stamp) && prev_msg) {
    const bool kSmart = false;
    auto cov = gtsam::noiseModel::Gaussian::Covariance(
        pos_receiver_cov_scale_ /
            (pos_msg->header.stamp - prev_msg->header.stamp).toSec() *
            Matrix3dRow::Map(pos_msg->position.covariance.data()),
        kSmart);
    gtsam::NonlinearFactor::shared_ptr pos_factor;
    if (estimate_antenna_positions_) {
      pos_factor = boost::make_shared<AbsolutePositionFactor2>(
          X(stamp_to_idx_[pos_msg->header.stamp]),
          P(stamp_to_idx_[pos_msg->header.stamp]), I_t_P, cov);
    } else {
      pos_factor = boost::make_shared<AbsolutePositionFactor1>(
          X(stamp_to_idx_[pos_msg->header.stamp]), I_t_P, B_t_P_, cov);
    }
    new_unary_factors_.emplace_back(stamp_to_idx_[pos_msg->header.stamp],
                                    pos_factor);
    solve();
  } else {
    ROS_WARN("Failed to add unary position factor.");
  }
  prev_msg = pos_msg;
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
  static piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr prev_msg =
      nullptr;
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
  } else if (addUnaryStamp(baseline_msg->header.stamp) && prev_msg) {
    auto cov = gtsam::noiseModel::Gaussian::Covariance(
        R_ENU_NED * att_receiver_cov_scale_ /
        (baseline_msg->header.stamp - prev_msg->header.stamp).toSec() *
        Matrix3dRow::Map(baseline_msg->position.covariance.data()) *
        R_ENU_NED.transpose());
    gtsam::NonlinearFactor::shared_ptr baseline_factor;
    if (estimate_antenna_positions_) {
      baseline_factor = boost::make_shared<MovingBaselineFactor2>(
          X(stamp_to_idx_[baseline_msg->header.stamp]),
          P(stamp_to_idx_[baseline_msg->header.stamp]),
          A(stamp_to_idx_[baseline_msg->header.stamp]), I_t_PA, cov);
    } else {
      baseline_factor = boost::make_shared<MovingBaselineFactor1>(
          X(stamp_to_idx_[baseline_msg->header.stamp]), I_t_PA, B_t_PA, cov);
    }
    new_unary_factors_.emplace_back(stamp_to_idx_[baseline_msg->header.stamp],
                                    baseline_factor);
    solve();
  } else {
    ROS_WARN("Failed to add unary baseline factor.");
  }
  prev_msg = baseline_msg;
}

void MavStateEstimator::publishOdometry(
    const Eigen::Vector3d& v_I, const Eigen::Vector3d& omega_B,
    const gtsam::imuBias::ConstantBias& bias, const ros::Time& stamp,
    const geometry_msgs::TransformStamped& T_IB) const {
  assert(T_IB.header.stamp == stamp);
  assert(T_IB.header.frame_id == inertial_frame_);
  assert(T_IB.child_frame_id == base_frame_);

  nav_msgs::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = inertial_frame_;
  msg.child_frame_id = "FLU";  // Publish odometry in FLU frame.

  try {
    geometry_msgs::TransformStamped T_BF;
    T_BF = tf_buffer_.lookupTransform(base_frame_, msg.child_frame_id, stamp);
    tf2::Stamped<tf2::Transform> tf2_T_IB, tf2_T_BF;
    tf2::fromMsg(T_IB, tf2_T_IB);
    tf2::fromMsg(T_BF, tf2_T_BF);

    // Pose
    auto tf2_T_IF = tf2_T_IB * tf2_T_BF;

    auto t_IF = tf2::toMsg(tf2_T_IF.getOrigin());
    Eigen::Vector3d p_IF;
    tf::vectorMsgToEigen(t_IF, p_IF);
    tf::pointEigenToMsg(p_IF, msg.pose.pose.position);

    msg.pose.pose.orientation = tf2::toMsg(tf2_T_IF.getRotation());

    // Twist
    tf::vectorEigenToMsg(v_I, msg.twist.twist.linear);
    tf2::Vector3 tf_v_I;
    tf2::fromMsg(msg.twist.twist.linear, tf_v_I);
    // TODO(rikba): Account for possible lever arm omega x r
    msg.twist.twist.linear =
        tf2::toMsg(tf2::quatRotate(tf2_T_IF.getRotation().inverse(), tf_v_I));

    auto omega_B_corrected = omega_B - bias.gyroscope();
    tf::vectorEigenToMsg(omega_B_corrected, msg.twist.twist.angular);
    tf2::Vector3 tf_omega_B;
    tf2::fromMsg(msg.twist.twist.angular, tf_omega_B);
    msg.twist.twist.angular = tf2::toMsg(
        tf2::quatRotate(tf2_T_BF.getRotation().inverse(), tf_omega_B));
  } catch (...) {
    ROS_WARN_THROTTLE(10.0,
                      "No odometry. Cannot lookup T_BF from B: %s to F: %s",
                      base_frame_.c_str(), msg.child_frame_id.c_str());
  }

  odometry_pub_.publish(msg);
}

geometry_msgs::TransformStamped MavStateEstimator::getTransform(
    const gtsam::NavState& state, const ros::Time& stamp,
    const std::string& child_frame_id) const {
  geometry_msgs::TransformStamped T_IB;
  T_IB.header.stamp = stamp;
  T_IB.header.frame_id = inertial_frame_;
  T_IB.child_frame_id = child_frame_id;

  tf::vectorEigenToMsg(state.position(), T_IB.transform.translation);
  tf::quaternionEigenToMsg(state.attitude().toQuaternion(),
                           T_IB.transform.rotation);

  return T_IB;
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

void MavStateEstimator::createBiasMessage(
    const gtsam::imuBias::ConstantBias& bias, const ros::Time& stamp,
    geometry_msgs::Vector3Stamped* acc_bias,
    geometry_msgs::Vector3Stamped* gyro_bias) const {
  assert(acc_bias);
  assert(gyro_bias);

  acc_bias->header.stamp = stamp;
  gyro_bias->header.stamp = stamp;

  tf::vectorEigenToMsg(bias.accelerometer(), acc_bias->vector);
  tf::vectorEigenToMsg(bias.gyroscope(), gyro_bias->vector);
}

void MavStateEstimator::publishBias(
    const gtsam::imuBias::ConstantBias& bias, const ros::Time& stamp,
    const ros::Publisher& acc_bias_pub, const ros::Publisher& gyro_bias_pub,
    geometry_msgs::Vector3Stamped* acc_bias,
    geometry_msgs::Vector3Stamped* gyro_bias) const {
  assert(acc_bias);
  assert(gyro_bias);
  createBiasMessage(bias, stamp, acc_bias, gyro_bias);

  acc_bias_pub.publish(*acc_bias);
  gyro_bias_pub.publish(*gyro_bias);
}

void MavStateEstimator::publishBias(const gtsam::imuBias::ConstantBias& bias,
                                    const ros::Time& stamp,
                                    const ros::Publisher& acc_bias_pub,
                                    const ros::Publisher& gyro_bias_pub) const {
  geometry_msgs::Vector3Stamped acc_bias, gyro_bias;
  publishBias(bias, stamp, acc_bias_pub, gyro_bias_pub, &acc_bias, &gyro_bias);
}

MavStateEstimator::~MavStateEstimator() {
  if (solver_thread_.joinable()) solver_thread_.join();
  if (batch_thread_.joinable()) batch_thread_.join();
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
  auto stamps = std::make_unique<gtsam::FixedLagSmoother::KeyTimestampMap>(
      new_timestamps_);
  auto time = std::make_unique<ros::Time>(idx_to_stamp_[idx_]);
  new_graph_.resize(0);
  new_values_.clear();
  new_timestamps_.clear();

  // Solve.
  is_solving_.store(true);
  solver_thread_ =
      std::thread(&MavStateEstimator::solveThreaded, this, std::move(graph),
                  std::move(values), std::move(stamps), std::move(time), idx_);
}

void MavStateEstimator::solveThreaded(
    std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
    std::unique_ptr<gtsam::Values> values,
    std::unique_ptr<gtsam::FixedLagSmoother::KeyTimestampMap> stamps,
    std::unique_ptr<ros::Time> time, const uint64_t i) {
  assert(graph);
  assert(values);
  assert(stamps);
  assert(time);

  // Solve iterative problem.
  gttic_(solveThreaded);
  smoother_.update(*graph, *values, *stamps);

  auto pose = smoother_.calculateEstimate<gtsam::Pose3>(X(i));
  auto velocity = smoother_.calculateEstimate<gtsam::Velocity3>(V(i));
  auto bias = smoother_.calculateEstimate<gtsam::imuBias::ConstantBias>(B(i));
  gtsam::Point3 B_t_P = B_t_P_;
  gtsam::Point3 B_t_A = B_t_A_;
  if (estimate_antenna_positions_) {
    B_t_P = smoother_.calculateEstimate<gtsam::Point3>(P(i));
    B_t_A = smoother_.calculateEstimate<gtsam::Point3>(A(i));
  }
  gttoc_(solveThreaded);
  gtsam::tictoc_finishedIteration_();

  // Update new values (threadsafe, blocks all sensor callbacks).
  gtsam::NavState new_state(pose, velocity);
  std::unique_lock<std::recursive_mutex> lock(update_mtx_);
  {
    prev_state_ = new_state;
    prev_bias_ = bias;

    if (estimate_antenna_positions_) {
      B_t_P_ = B_t_P;
      B_t_A_ = B_t_A;
    }

    auto idx = i;
    for (auto it = new_graph_.begin(); it != new_graph_.end(); ++it) {
      auto imu = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(*it);
      if (imu) {
        prev_state_ =
            imu->preintegratedMeasurements().predict(prev_state_, prev_bias_);
        new_values_.update(X(idx + 1), prev_state_.pose());
        new_values_.update(V(idx + 1), prev_state_.velocity());
        new_values_.update(B(idx + 1), prev_bias_);
        if (estimate_antenna_positions_) {
          new_values_.update(P(idx + 1), B_t_P_);
          new_values_.update(A(idx + 1), B_t_A_);
        }
        idx++;
      }
    }
  }

  // ROS publishers
  tictoc_getNode(solveThreaded, solveThreaded);
  timing_msg_.header.stamp = *time;
  timing_msg_.iteration = solveThreaded->self() - timing_msg_.time;
  timing_msg_.time = solveThreaded->self();
  timing_msg_.min = solveThreaded->min();
  timing_msg_.max = solveThreaded->max();
  timing_msg_.mean = solveThreaded->mean();
  timing_pub_.publish(timing_msg_);

  geometry_msgs::TransformStamped T_IB =
      getTransform(new_state, *time, base_frame_ + "_optimization");
  tfb_.sendTransform(T_IB);
  optimization_pub_.publish(T_IB);
  publishBias(bias, *time, acc_bias_pub_, gyro_bias_pub_);
  publishAntennaPosition(B_t_P, *time, position_antenna_pub_);
  publishAntennaPosition(B_t_A, *time, attitude_antenna_pub_);

  // Update batch graph.
  // TODO(rikba): Maybe filter out prior factors.
  {
    std::unique_lock<std::mutex> lock(batch_mtx_);
    batch_graph_.push_back(graph->begin(), graph->end());
    batch_values_.insert(*values);
    batch_values_.update(X(i), pose);
    batch_values_.update(V(i), velocity);
    batch_values_.update(B(i), bias);
    if (estimate_antenna_positions_) {
      batch_values_.update(P(i), B_t_P);
      batch_values_.update(A(i), B_t_A);
    }
  }

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
  // ROS_INFO_STREAM("Computation time " << timing_msg_.iteration);

  is_solving_.store(false);
}

bool MavStateEstimator::computeBatchSolution(Batch::Request& request,
                                             Batch::Response& response) {
  auto already_running = batch_running_.load();
  bool initialized = batch_graph_.size();
  if (!already_running && initialized) {
    batch_running_.store(true);
    // Create deep copies of current graph and initial values.
    {
      std::unique_lock<std::mutex> lock(batch_mtx_);

      auto graph = std::make_unique<gtsam::NonlinearFactorGraph>(batch_graph_);
      auto values = std::make_unique<gtsam::Values>(batch_values_);
      auto imus =
          std::make_unique<std::vector<sensor_msgs::Imu::ConstPtr>>(batch_imu_);
      auto integrator =
          std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
              integrator_);
      auto idx_to_stamp =
          std::make_unique<std::map<uint64_t, ros::Time>>(idx_to_stamp_);
      auto bag_file = std::make_unique<std::string>(request.bag_file);

      if (batch_thread_.joinable()) batch_thread_.join();
      batch_thread_ =
          std::thread(&MavStateEstimator::solveBatch, this, std::move(graph),
                      std::move(values), std::move(imus), std::move(integrator),
                      std::move(idx_to_stamp), std::move(bag_file));
    }
  }

  return !already_running && initialized;
}

void MavStateEstimator::solveBatch(
    std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
    std::unique_ptr<gtsam::Values> values,
    std::unique_ptr<std::vector<sensor_msgs::Imu::ConstPtr>> imus,
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> integrator,
    std::unique_ptr<std::map<uint64_t, ros::Time>> idx_to_stamp,
    std::unique_ptr<std::string> bag_file) {
  assert(graph);
  assert(values);
  assert(imu);
  assert(integrator);
  assert(idx_to_stamp);
  assert(bag_file);
  ROS_INFO("Solving batch with %lu factors and %lu values.", graph->size(),
           values->size());

  rosbag::Bag bag;
  try {
    bag.open(*bag_file, rosbag::bagmode::Write);
  } catch (const rosbag::BagException& e) {
    ROS_WARN("Cannot open bag file %s: %s", bag_file->c_str(), e.what());
  }

  const std::string kAccBiasTopic =
      nh_private_.getNamespace() + "/batch_acc_bias";
  const std::string kGyroBiasTopic =
      nh_private_.getNamespace() + "/batch_gyro_bias";
  const std::string kTfTopic = nh_private_.getNamespace() + "/batch";

  ros::Publisher status_pub;
  const uint32_t kBatchStatusQueueSize = 1;
  status_pub = nh_private_.advertise<mav_state_estimation::BatchStatus>(
      "batch_status", kBatchStatusQueueSize);
  mav_state_estimation::BatchStatus batch_status;
  batch_status.total_idx = idx_to_stamp->size();
  batch_status.finished = false;

  gtsam::LevenbergMarquardtOptimizer optimizer(*graph, *values);
  auto result = optimizer.optimize();

  try {
    uint64_t idx = 0;
    batch_status.current_idx = idx;
    auto start = idx_to_stamp->at(idx);
    auto prev_imu = std::find_if(imus->begin(), imus->end(),
                                 [&start](const sensor_msgs::Imu::ConstPtr& x) {
                                   return x->header.stamp == start;
                                 });
    if (prev_imu == imus->end()) {
      ROS_ERROR("Cannot not find start IMU message.");
    } else {
      gtsam::NavState prev_state(result.at<gtsam::Pose3>(X(idx)),
                                 result.at<gtsam::Velocity3>(V(idx)));
      auto prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(idx));
      integrator->resetIntegrationAndSetBias(prev_bias);
      for (; prev_imu != std::prev(imus->end()); ++prev_imu) {
        auto imu = std::next(prev_imu);
        Eigen::Vector3d lin_acc, ang_vel;
        tf::vectorMsgToEigen((*prev_imu)->linear_acceleration, lin_acc);
        tf::vectorMsgToEigen((*prev_imu)->angular_velocity, ang_vel);
        auto dt = ((*imu)->header.stamp - (*prev_imu)->header.stamp).toSec();
        integrator->integrateMeasurement(lin_acc, ang_vel, dt);
        auto imu_state = integrator->predict(prev_state, prev_bias);

        // Update solution index.
        if ((*imu)->header.stamp == idx_to_stamp->at(idx + 1)) {
          idx++;
          batch_status.current_idx = idx;
          // Publish status every 10 percent.
          if (((10 * batch_status.current_idx) % batch_status.total_idx) == 0) {
            status_pub.publish(batch_status);
          }
          try {
            prev_state = gtsam::NavState(result.at<gtsam::Pose3>(X(idx)),
                                         result.at<gtsam::Velocity3>(V(idx)));
            imu_state = prev_state;
            prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(idx));
            integrator->resetIntegrationAndSetBias(prev_bias);
            try {
              geometry_msgs::Vector3Stamped acc_bias, gyro_bias;
              createBiasMessage(prev_bias, (*imu)->header.stamp, &acc_bias,
                                &gyro_bias);
              bag.write(kAccBiasTopic, acc_bias.header.stamp, acc_bias);
              bag.write(kGyroBiasTopic, gyro_bias.header.stamp, gyro_bias);
            } catch (const rosbag::BagIOException& e) {
              ROS_WARN_ONCE("Cannot write batch bias to bag: %s", e.what());
            } catch (const rosbag::BagException& e) {
              ROS_WARN("Cannot write batch bias: %s", e.what());
            }
          } catch (const gtsam::ValuesKeyDoesNotExist& e) {
            ROS_WARN_STREAM("Missing value at index: "
                            << idx << ", stamp: " << idx_to_stamp->at(idx)
                            << " error: " << e.what());
          }
        } else if ((*imu)->header.stamp > idx_to_stamp->at(idx + 1)) {
          ROS_ERROR("Cannot find unary IMU message.");
          break;
        }

        // Log
        geometry_msgs::TransformStamped T_IB =
            getTransform(imu_state, (*imu)->header.stamp, base_frame_);
        try {
          bag.write(kTfTopic, T_IB.header.stamp, T_IB);
        } catch (const rosbag::BagIOException& e) {
          ROS_WARN_ONCE("Cannot write batch pose to bag: %s", e.what());
        } catch (const rosbag::BagException& e) {
          ROS_WARN("Cannot write batch pose: %s", e.what());
        }

        // Additional external poses
        tf2::Stamped<tf2::Transform> tf2_T_IB;
        tf2::fromMsg(T_IB, tf2_T_IB);
        for (const auto& frame : external_poses_) {
          geometry_msgs::TransformStamped T_BE;
          try {
            T_BE = tf_buffer_.lookupTransform(base_frame_, frame,
                                              T_IB.header.stamp);
          } catch (...) {
            ROS_WARN_ONCE("Cannot lookup T_BE from B: %s E: %s",
                          base_frame_.c_str(), frame.c_str());
            continue;
          }
          tf2::Stamped<tf2::Transform> tf2_T_BE;
          tf2::fromMsg(T_BE, tf2_T_BE);
          auto tf2_T_IE = tf2_T_IB * tf2_T_BE;

          geometry_msgs::TransformStamped T_IE;
          T_IE.transform = tf2::toMsg(tf2_T_IE);
          T_IE.header.stamp = T_IB.header.stamp;
          T_IE.header.frame_id = inertial_frame_;
          T_IE.child_frame_id = frame;

          try {
            bag.write(frame, T_IE.header.stamp, T_IE);
          } catch (const rosbag::BagIOException& e) {
            ROS_WARN_ONCE("Cannot write batch pose %s to bag: %s",
                          frame.c_str(), e.what());
          } catch (const rosbag::BagException& e) {
            ROS_WARN("Cannot write batch pose %s: %s", frame.c_str(), e.what());
          }
        }
      }
      ROS_INFO_STREAM("Solved batch solution from "
                      << start << " to " << (*prev_imu)->header.stamp);
    }
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Unary index out of range: %s", e.what());
  }

  bag.close();
  batch_running_.store(false);

  batch_status.finished = true;
  status_pub.publish(batch_status);
}

}  // namespace mav_state_estimation
