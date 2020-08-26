#ifndef MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
#define MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_

#include <atomic>
#include <mutex>
#include <thread>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

#include "mav_state_estimation/Timing.h"
#include "mav_state_estimation/initialization.h"

namespace mav_state_estimation {

class MavStateEstimator {
 public:
  MavStateEstimator();
  ~MavStateEstimator();

 private:
  Eigen::Vector3d getVectorFromParams(const std::string& param) const;

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void posCallback(
      const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr& pos_msg);
  void baselineCallback(
      const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr&
          baseline_msg);

  void broadcastTf(const gtsam::NavState& state, const ros::Time& stamp,
                   const std::string& child_frame_id);
  void publishPose(const gtsam::NavState& state, const ros::Time& stamp,
                   const ros::Publisher& pub) const;
  void publishBias(const gtsam::imuBias::ConstantBias& bias,
                   const ros::Time& stamp) const;

  void addSensorTimes(const uint16_t rate);
  bool addUnaryStamp(const ros::Time& stamp);
  gtsam::imuBias::ConstantBias getCurrentBias();
  gtsam::NavState getCurrentState();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Subscriber pos_0_sub_;
  ros::Subscriber baseline_sub_;

  ros::Publisher timing_pub_;
  mav_state_estimation::Timing timing_msg_;

  ros::Publisher prediction_pub_;
  ros::Publisher optimization_pub_;
  ros::Publisher acc_bias_pub_;
  ros::Publisher gyro_bias_pub_;

  tf2_ros::TransformBroadcaster tfb_;

  // Initial parameters.
  Initialization init_;
  Eigen::Vector3d B_t_P_ = Eigen::Vector3d::Zero();  // Position receiver.
  Eigen::Vector3d B_t_A_ = Eigen::Vector3d::Zero();  // Attitude receiver.
  std::string inertial_frame_;
  std::string base_frame_;

  // GTSAM variables.
  void initializeState();
  std::set<uint32_t> unary_times_ns_;
  std::map<ros::Time, uint32_t> stamp_to_idx_;
  std::map<uint32_t, ros::Time> idx_to_stamp_;
  std::map<ros::Time, uint32_t>::iterator next_imu_factor_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_T_I_B_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_I_v_B_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_imu_bias_;
  gtsam::PreintegratedCombinedMeasurements imu_integration_;
  std::vector<gtsam::NonlinearFactor::shared_ptr> new_factors_;
  std::deque<std::pair<uint32_t, gtsam::NonlinearFactor::shared_ptr>>
      new_unary_factors_;
  sensor_msgs::Imu::ConstPtr prev_imu_;
  gtsam::NavState prev_unary_state_;

  // Extra thread to solve factor graph.
  std::thread solver_thread_;
  std::atomic_bool is_solving_ = false;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::LevenbergMarquardtOptimizer::shared_ptr optimizer_;

  // Mutex lock initial values which are accessed by all threads.
  void updateInitialValues();
  gtsam::Values initial_values_;
  std::recursive_mutex update_mtx_;
  template <class ValueType>
  inline ValueType getInitialValue(const gtsam::Key& key) {
    try {
      return initial_values_.at<ValueType>(key);
    } catch (const std::out_of_range& e) {
      ROS_ERROR("Index out of range: %s", e.what());
      return ValueType();
    } catch (const std::bad_cast& e) {
      ROS_ERROR("Cannot cast object: %s", e.what());
      return ValueType();
    }
  }

  inline size_t getLastIdx() {
    return gtsam::symbolIndex(initial_values_.rbegin()->key);
  }

  void solve();

  void solveThreaded();
};

}  // namespace mav_state_estimation

#endif  // MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
