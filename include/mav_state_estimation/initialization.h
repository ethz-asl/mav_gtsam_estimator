#ifndef MAV_STATE_ESTIMATION_INITIALIZATION_H_
#define MAV_STATE_ESTIMATION_INITIALIZATION_H_

#include <Eigen/Dense>
#include <optional>
#include <tf2_ros/transform_broadcaster.h>

namespace mav_state_estimation {

// Orientation (and position) initialization using the TRIAD method.
class Initialization {
public:
  void addOrientationConstraint(const Eigen::Vector3d r_B,
                                const Eigen::Vector3d r_I);
  void addPositionConstraint(const Eigen::Vector3d r_I,
                             const Eigen::Vector3d B_t_P);

private:
  void computeInitialState();

  std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> r_1;
  std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> r_2;
  std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> p;

  tf2_ros::TransformBroadcaster br_;
};

}
#endif // MAV_STATE_ESTIMATION_INITIALIZATION_H_
