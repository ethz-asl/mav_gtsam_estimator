#ifndef MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
#define MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_

#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <piksi_rtk_msgs/VelocityWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace mav_state_estimation {

class MavStateEstimator {
public:
  MavStateEstimator();

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);
  void posCallback(
      const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr &pos_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Subscriber pos_0_sub_;
};

} // namespace mav_state_estimator

#endif // MAV_STATE_ESTIMATOR_MAV_STATE_ESTIMATOR_H_
