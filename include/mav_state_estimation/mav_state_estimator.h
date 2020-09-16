#ifndef MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
#define MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_

#include <atomic>
#include <mutex>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

#include "mav_state_estimation/Batch.h"
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

  geometry_msgs::TransformStamped getTransform(
      const gtsam::NavState& state, const ros::Time& stamp,
      const std::string& child_frame_id) const;
  void broadcastTf(const gtsam::NavState& state, const ros::Time& stamp,
                   const std::string& child_frame_id);
  void broadcastTf(const gtsam::NavState& state, const ros::Time& stamp,
                   const std::string& child_frame_id,
                   geometry_msgs::TransformStamped* T_IB);
  void publishOdometry(const Eigen::Vector3d& v_I,
                       const Eigen::Vector3d& omega_B,
                       const gtsam::imuBias::ConstantBias& bias,
                       const ros::Time& stamp,
                       const geometry_msgs::TransformStamped& T_IB) const;
  void publishBias(const gtsam::imuBias::ConstantBias& bias,
                   const ros::Time& stamp, const ros::Publisher& acc_bias_pub,
                   const ros::Publisher& gyro_bias_pub) const;
  void publishBias(const gtsam::imuBias::ConstantBias& bias,
                   const ros::Time& stamp, const ros::Publisher& acc_bias_pub,
                   const ros::Publisher& gyro_bias_pub,
                   geometry_msgs::Vector3Stamped* acc_bias,
                   geometry_msgs::Vector3Stamped* gyro_bias) const;
  void publishAntennaPosition(const gtsam::Point3& B_t, const ros::Time& stamp,
                              const ros::Publisher& pub) const;

  void loadGnssParams(const std::string& antenna_ns,
                      const gtsam::Symbol& symbol, gtsam::Point3* B_t,
                      gtsam::noiseModel::Isotropic::shared_ptr* process_noise,
                      double* cov_scale);

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
  ros::Publisher position_antenna_pub_;
  ros::Publisher attitude_antenna_pub_;
  ros::Publisher odometry_pub_;

  ros::ServiceServer batch_srv_;

  tf2_ros::TransformBroadcaster tfb_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tfl_ = tf2_ros::TransformListener(tf_buffer_);
  std::vector<std::string> external_poses_;

  // Initial parameters.
  Initialization init_;
  gtsam::Point3 B_t_P_ = Eigen::Vector3d::Zero();  // Position receiver.
  gtsam::Point3 B_t_A_ = Eigen::Vector3d::Zero();  // Attitude receiver.
  gtsam::noiseModel::Isotropic::shared_ptr process_noise_model_B_t_P_;
  gtsam::noiseModel::Isotropic::shared_ptr process_noise_model_B_t_A_;
  std::string inertial_frame_;
  std::string base_frame_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_T_I_B_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_I_v_B_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_imu_bias_;
  bool estimate_antenna_positions_ = false;

  double pos_receiver_cov_scale_ = 1.0;
  double att_receiver_cov_scale_ = 1.0;

  void initializeState();
  inline bool isInitialized() const { return !stamp_to_idx_.empty(); }
  std::set<uint32_t> unary_times_ns_;
  std::map<ros::Time, uint64_t> stamp_to_idx_;
  std::map<uint64_t, ros::Time> idx_to_stamp_;

  // IMU interation
  gtsam::PreintegratedCombinedMeasurements integrator_;
  sensor_msgs::Imu::ConstPtr prev_imu_;

  // ISAM2
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph new_graph_;
  gtsam::Values new_values_;
  uint64_t idx_ = 0;
  std::vector<std::pair<uint64_t, gtsam::NonlinearFactor::shared_ptr>>
      new_unary_factors_;
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  int max_window_ = -1;
  gtsam::FastList<gtsam::Key> no_relin_keys_;
  uint64_t next_elim_idx_ = 0;

  // Batch
  std::mutex batch_mtx_;
  gtsam::NonlinearFactorGraph batch_graph_;
  gtsam::Values batch_values_;
  std::vector<sensor_msgs::Imu::ConstPtr> batch_imu_;
  void solveBatch(
      std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
      std::unique_ptr<gtsam::Values> values,
      std::unique_ptr<std::vector<sensor_msgs::Imu::ConstPtr>> imus,
      std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> integrator,
      std::unique_ptr<std::map<uint64_t, ros::Time>> idx_to_stamp,
      std::unique_ptr<std::string> bag_file);
  bool computeBatchSolution(Batch::Request& request, Batch::Response& response);
  std::thread batch_thread_;
  std::atomic_bool batch_running_ = false;

  // Extra thread to solve factor graph.
  std::thread solver_thread_;
  std::atomic_bool is_solving_ = false;
  std::recursive_mutex update_mtx_;

  void solve();

  void solveThreaded(std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
                     std::unique_ptr<gtsam::Values> values,
                     std::unique_ptr<ros::Time> time, const uint64_t i);
};

}  // namespace mav_state_estimation

#endif  // MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
