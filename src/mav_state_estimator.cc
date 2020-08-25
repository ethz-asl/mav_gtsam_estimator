#include "mav_state_estimation/mav_state_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/Symbol.h>
#include <ros/ros.h>

#include "mav_state_estimation/Timing.h"
#include "mav_state_estimation/absolute_position_factor.h"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
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

  B_t_A_ = getVectorFromParams("attitude_receiver/B_t");
  ROS_INFO_STREAM("Initial guess B_t_A: " << B_t_A_.transpose());
  nh_private_.getParam("attitude_receiver/rate", rate);
  addSensorTimes(rate);

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
  initial_values_.insert(
      B(0), gtsam::imuBias::ConstantBias(prior_acc_bias, prior_gyro_bias));
  initial_values_.at<gtsam::imuBias::ConstantBias>(B(0)).print(
      "prior_imu_bias: ");

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
  imu_integration_ =
      gtsam::PreintegratedCombinedMeasurements(imu_params, getCurrentBias());

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
    Eigen::Vector3d I_t_B;
    Eigen::Quaterniond q_IB;
    tf::vectorMsgToEigen(T_IB_0.transform.translation, I_t_B);
    tf::quaternionMsgToEigen(T_IB_0.transform.rotation, q_IB);
    Eigen::Vector3d I_v_B = Eigen::Vector3d::Zero();

    gtsam::Pose3 T_IB(gtsam::Rot3(q_IB), I_t_B);
    initial_values_.insert(X(0), T_IB);
    initial_values_.insert(V(0), I_v_B);

    inertial_frame_ = T_IB_0.header.frame_id;
    base_frame_ = T_IB_0.child_frame_id;
    stamp_to_idx_[T_IB_0.header.stamp] = 0;
    idx_to_stamp_[stamp_to_idx_[T_IB_0.header.stamp]] = T_IB_0.header.stamp;
    auto first_unary_time = T_IB_0.header.stamp;
    first_unary_time.nsec =
        *unary_times_ns_.upper_bound(T_IB_0.header.stamp.nsec);
    bool initialized_stamps = addUnaryStamp(first_unary_time);
    assert(initialized_stamps);
    initial_values_.print("Initial state: ");
    ROS_INFO_STREAM("Initialization stamp: " << T_IB_0.header.stamp);
    next_imu_factor_ = std::next(stamp_to_idx_.begin());
    assert(next_imu_factor_ != stamp_to_idx_.end());
    ROS_INFO_STREAM("Next unary stamp: " << next_imu_factor_->first);
  }
}

gtsam::imuBias::ConstantBias MavStateEstimator::getCurrentBias() const {
  try {
    return initial_values_.at<gtsam::imuBias::ConstantBias>(
        B(gtsam::symbolIndex(initial_values_.rbegin()->key)));
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Bias index out of range: %s", e.what());
    return gtsam::imuBias::ConstantBias();
  } catch (const std::bad_cast& e) {
    ROS_ERROR("Cannot cast bias: %s", e.what());
    return gtsam::imuBias::ConstantBias();
  }
}

gtsam::NavState MavStateEstimator::getCurrentState() const {
  try {
    auto idx = gtsam::symbolIndex(initial_values_.rbegin()->key);
    gtsam::NavState nav_state(initial_values_.at<gtsam::Pose3>(X(idx)),
                              initial_values_.at<gtsam::Velocity3>(V(idx)));
    return nav_state;
  } catch (const std::out_of_range& e) {
    ROS_ERROR("State index out of range: %s", e.what());
    return gtsam::NavState();
  } catch (const std::bad_cast& e) {
    ROS_ERROR("Cannot cast state: %s", e.what());
    return gtsam::NavState();
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
    initializeState();
  } else if (imu_msg->header.stamp > next_imu_factor_->first) {
    // Handle dropped IMU message.
    // TODO(rikba): Linearly inpterpolate IMU message.
    sensor_msgs::Imu in_between_imu = *prev_imu_;
    in_between_imu.header.stamp = next_imu_factor_->first;
    ROS_WARN("Inserting missing IMU message at %u.%u",
             in_between_imu.header.stamp.sec, in_between_imu.header.stamp.nsec);
    ROS_WARN_STREAM("Prev IMU stamp: " << prev_imu_->header.stamp);
    ROS_WARN_STREAM("This IMU stamp: " << imu_msg->header.stamp);
    imuCallback(boost::make_shared<sensor_msgs::Imu>(in_between_imu));
  } else if (imu_msg->header.stamp > prev_imu_->header.stamp) {
    // Integrate IMU (zero-order-hold).
    Eigen::Vector3d lin_acc, ang_vel;
    tf::vectorMsgToEigen(prev_imu_->linear_acceleration, lin_acc);
    tf::vectorMsgToEigen(prev_imu_->angular_velocity, ang_vel);
    double dt = (imu_msg->header.stamp - prev_imu_->header.stamp).toSec();
    imu_integration_.integrateMeasurement(lin_acc, ang_vel, dt);

    // Publish high rate IMU prediction.
    gtsam::NavState imu_state =
        imu_integration_.predict(getCurrentState(), getCurrentBias());
    broadcastTf(imu_state, imu_msg->header.stamp, base_frame_ + "_prediction");
    publishPose(imu_state, imu_msg->header.stamp, prediction_pub_);

    // Setup new inbetween IMU factor.
    if (addUnaryStamp(imu_msg->header.stamp)) {
      uint32_t idx = stamp_to_idx_[imu_msg->header.stamp];

      initial_values_.insert(B(idx), getCurrentBias());
      initial_values_.insert(X(idx), imu_state.pose());
      initial_values_.insert(V(idx), imu_state.v());

      gtsam::CombinedImuFactor::shared_ptr imu_factor =
          boost::make_shared<gtsam::CombinedImuFactor>(
              X(idx - 1), V(idx - 1), X(idx), V(idx), B(idx - 1), B(idx),
              imu_integration_);
      imu_integration_.resetIntegrationAndSetBias(getCurrentBias());
      new_factors_.push_back(imu_factor);
      next_imu_factor_ = std::next(next_imu_factor_);
      assert(next_imu_factor_ == stamp_to_idx_.end());

      // Transfer all new unary factors to new factors if IMU inbetween factor
      // exists already.
      auto it = new_unary_factors_.begin();
      while (it != new_unary_factors_.end() && it->first <= idx) {
        new_factors_.push_back(it->second);
        it = std::next(it);
        new_unary_factors_.pop_front();
      }

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
  ROS_INFO_ONCE("Received first POS message.");
  Eigen::Vector3d I_t_P;
  tf::pointMsgToEigen(pos_msg->position.position, I_t_P);
  if (!init_.isInitialized()) {
    // GNSS antenna position in inertial frame (ENU).
    init_.addPositionConstraint(I_t_P, B_t_P_, pos_msg->header.stamp);
    init_.setInertialFrame(pos_msg->header.frame_id);
    initializeState();
  } else if (addUnaryStamp(pos_msg->header.stamp)) {
    const uint32_t idx = stamp_to_idx_[pos_msg->header.stamp];
    const bool kSmart = false;
    auto cov = gtsam::noiseModel::Gaussian::Covariance(
        Matrix3dRow::Map(pos_msg->position.covariance.data()), kSmart);
    AbsolutePositionFactor::shared_ptr pos_factor =
        boost::make_shared<AbsolutePositionFactor>(X(idx), I_t_P, B_t_P_, cov);
    new_unary_factors_.emplace_back(idx, pos_factor);
  } else {
    ROS_ERROR("Failed to add unary position factor.");
  }
}

bool MavStateEstimator::addUnaryStamp(const ros::Time& stamp) {
  bool valid = (stamp >= stamp_to_idx_.begin()->first);
  ROS_ERROR_COND(!valid,
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
    initializeState();
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

MavStateEstimator::~MavStateEstimator() {
  if (solver_thread_.joinable()) solver_thread_.join();
}

void MavStateEstimator::solve() {
  // Check for new result.
  if (is_solving_.load()) {
    return;  // Still solving.
  } else if (!solver_thread_.joinable()) {
    ROS_INFO("Starting thread for the first time.");
  } else {
    solver_thread_.join();
    // Update initial states with recent optimization.
    initial_values_.update(optimizer_->values());
    auto idx = gtsam::symbolIndex(optimizer_->values().rbegin()->key);
    // Broadcast latest optimized pose.
    gtsam::NavState nav_prev(initial_values_.at<gtsam::Pose3>(X(idx)),
                             initial_values_.at<gtsam::Velocity3>(V(idx)));
    broadcastTf(nav_prev, idx_to_stamp_[idx], base_frame_ + "_optimization");
    publishPose(nav_prev, idx_to_stamp_[idx], optimization_pub_);

    // Publish solving time.
    tictoc_getNode(solveThreaded, solveThreaded);
    timing_msg_.header.stamp = idx_to_stamp_[idx];
    timing_msg_.iteration = solveThreaded->self() - timing_msg_.time;
    timing_msg_.time = solveThreaded->self();
    timing_msg_.min = solveThreaded->min();
    timing_msg_.max = solveThreaded->max();
    timing_msg_.mean = solveThreaded->mean();
    timing_pub_.publish(timing_msg_);

    // Update future states with new initial state and IMU bias.
    for (auto factor : new_factors_) {
      auto imu_factor =
          boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
      if (imu_factor) {
        try {
          gtsam::NavState nav_prev(
              initial_values_.at<gtsam::Pose3>(X(idx)),
              initial_values_.at<gtsam::Velocity3>(V(idx)));
          auto bias_prev =
              initial_values_.at<gtsam::imuBias::ConstantBias>(B(idx));
          auto nav_next = imu_factor->preintegratedMeasurements().predict(
              nav_prev, bias_prev);
          initial_values_.update(X(idx + 1), nav_next.pose());
          initial_values_.update(V(idx + 1), nav_next.velocity());
          initial_values_.update(B(idx + 1), bias_prev);
        } catch (const std::out_of_range& e) {
          ROS_ERROR("Index %lu out of range: %s.", idx, e.what());
        } catch (const std::bad_cast& e) {
          ROS_ERROR("Cannot cast values: %s", e.what());
        }
        idx++;
      }
    }
  }

  if (new_factors_.empty()) {
    ROS_WARN("No new factors.");
    return;
  }

  // Add new factors to graph.
  for (auto factor : new_factors_) {
    graph_.add(factor);
  }
  new_factors_.clear();

  // Solve.
  optimizer_ = boost::make_shared<gtsam::LevenbergMarquardtOptimizer>(
      graph_, initial_values_);
  is_solving_.store(true);
  solver_thread_ = std::thread(&MavStateEstimator::solveThreaded, this);
}

void MavStateEstimator::solveThreaded() {
  gttic_(solveThreaded);
  auto result = optimizer_->optimize();
  gttoc_(solveThreaded);
  gtsam::tictoc_finishedIteration_();
  is_solving_.store(false);
}

}  // namespace mav_state_estimation

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_state_estimator");
  mav_state_estimation::MavStateEstimator estimator;
  ros::spin();
  return 0;
}
