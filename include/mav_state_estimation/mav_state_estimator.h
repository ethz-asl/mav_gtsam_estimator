#ifndef MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
#define MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_

#include <Eigen/Dense>
#include <optional>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <piksi_rtk_msgs/VelocityWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>

namespace mav_state_estimation {

class MavStateEstimator {
public:
  MavStateEstimator();

private:
  Eigen::Vector3d getVectorFromParams(const std::string &param) const;

  void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);
  void posCallback(
      const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr &pos_msg);
  void
  baselineCallback(const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr
                       &baseline_msg);

  void calculateTriadOrientation(const Eigen::Vector3d &R1,
                                 const Eigen::Vector3d &r1,
                                 const Eigen::Vector3d &R2,
                                 const Eigen::Vector3d &r2,
                                 Eigen::Quaterniond *q) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Subscriber pos_0_sub_;
  ros::Subscriber baseline_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tfl = tf2_ros::TransformListener(tf_buffer_);

  // State initialization.
  Eigen::Vector3d B_t_P_ = Eigen::Vector3d::Zero(); // Position receiver.
  Eigen::Vector3d B_t_A_ = Eigen::Vector3d::Zero(); // Attitude receiver.
  std::optional<Eigen::Vector3d> init_g_imu_;
  std::optional<Eigen::Vector3d> heading_init_;
  std::optional<Eigen::Vector3d> pos_init_;
};

} // namespace mav_state_estimation

#endif // MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
