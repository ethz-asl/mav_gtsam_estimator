#ifndef MAV_STATE_ESTIMATION_INITIALIZATION_H_
#define MAV_STATE_ESTIMATION_INITIALIZATION_H_

#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <optional>

namespace mav_state_estimation {

// Orientation (and position) initialization using the TRIAD method.
class Initialization {
 public:
  // Add a vector measurement both in inertial and base frame.
  void addOrientationConstraint1(const Eigen::Vector3d& r_I,
                                 const Eigen::Vector3d& r_B,
                                 const ros::Time& t);
  // Add a second vector measurement both in inertial and base frame.
  void addOrientationConstraint2(const Eigen::Vector3d& r_I,
                                 const Eigen::Vector3d& r_B,
                                 const ros::Time& t);
  // Add a position constraint and its corresponding transformation from base
  // frame to position sensor.
  void addPositionConstraint(const Eigen::Vector3d& I_r_P,
                             const Eigen::Vector3d& B_t_P, const ros::Time& t);

  inline void setInertialFrame(const std::string id) {
    inertial_frame_ = std::make_optional<std::string>(id);
  }
  inline void setBaseFrame(const std::string id) {
    base_frame_ = std::make_optional<std::string>(id);
  }

  inline bool isInitialized() const { return T_IB_.has_value(); };
  bool getInitialPose(geometry_msgs::TransformStamped* T_IB) const;

 private:
  void computeInitialState();

  void calculateTriadOrientation(const Eigen::Vector3d& R1,
                                 const Eigen::Vector3d& r1,
                                 const Eigen::Vector3d& R2,
                                 const Eigen::Vector3d& r2,
                                 Eigen::Matrix3d* R_IB) const;

  typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> VectorPair;
  std::optional<VectorPair> r_1;
  std::optional<VectorPair> r_2;
  std::optional<VectorPair> p;
  ros::Time stamp_;
  std::optional<std::string> inertial_frame_;
  std::optional<std::string> base_frame_;
  std::optional<geometry_msgs::TransformStamped> T_IB_;
};

}  // namespace mav_state_estimation
#endif  // MAV_STATE_ESTIMATION_INITIALIZATION_H_
