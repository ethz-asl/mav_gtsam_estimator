#include "mav_state_estimation/initialization.h"

#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <ros/assert.h>

namespace mav_state_estimation {

void Initialization::addOrientationConstraint1(const Eigen::Vector3d& r_I,
                                               const Eigen::Vector3d& r_B,
                                               const ros::Time& t) {
  r_1 = std::make_optional<VectorPair>(r_I, r_B);
  stamp_ = t;
  computeInitialState();
}

void Initialization::addOrientationConstraint2(const Eigen::Vector3d& r_I,
                                               const Eigen::Vector3d& r_B,
                                               const ros::Time& t) {
  r_2 = std::make_optional<VectorPair>(r_I, r_B);
  stamp_ = t;
  computeInitialState();
}

void Initialization::addPositionConstraint(const Eigen::Vector3d& I_t_P,
                                           const Eigen::Vector3d& B_t_P,
                                           const ros::Time& t) {
  p = std::make_optional<VectorPair>(I_t_P, B_t_P);
  stamp_ = t;
  computeInitialState();
}

bool Initialization::getInitialPose(
    geometry_msgs::TransformStamped* T_IB) const {
  bool is_initialized = T_IB_.has_value();
  if (is_initialized) {
    *T_IB = *T_IB_;
  }
  return is_initialized;
}

void Initialization::computeInitialState() {
  bool is_initialized = r_1 && r_2 && p;
  if (is_initialized) {
    Eigen::Matrix3d R_IB;
    calculateTriadOrientation(r_1->first, r_1->second, r_2->first, r_2->second,
                              &R_IB);
    Eigen::Vector3d I_t_B = p->first - R_IB * p->second;
    T_IB_ = std::make_optional<geometry_msgs::TransformStamped>();
    T_IB_->header.stamp = stamp_;
    tf::vectorEigenToMsg(I_t_B, T_IB_->transform.translation);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(R_IB),
                             T_IB_->transform.rotation);
  } else {
    ROS_DEBUG("Missing initial conditions.");
  }
}

void Initialization::calculateTriadOrientation(const Eigen::Vector3d& R1,
                                               const Eigen::Vector3d& r1,
                                               const Eigen::Vector3d& R2,
                                               const Eigen::Vector3d& r2,
                                               Eigen::Matrix3d* R_IB) const {
  ROS_ASSERT(R_IB);

  // Calculate the rotation between coordinate frames given two linearly
  // independent measurements. https://en.wikipedia.org/wiki/Triad_method
  Eigen::Vector3d S = R1.normalized();
  Eigen::Vector3d s = r1.normalized();
  Eigen::Vector3d M = (R1.cross(R2)).normalized();
  Eigen::Vector3d m = (r1.cross(r2)).normalized();

  Eigen::Matrix3d a;
  a << s, m, s.cross(m);
  ROS_INFO_STREAM("\n" << a);
  Eigen::Matrix3d b;
  b << S, M, S.cross(M);
  ROS_INFO_STREAM("\n" << b);

  *R_IB = b * a.transpose();
  //*q = Eigen::Quaterniond(A);
  // ROS_INFO("Computed initial orientation q_IB [x, y, z, w]: [%.2f, %.2f,
  // %.2f, "
  //         "%.2f]",
  //         q->x(), q->y(), q->z(), q->w());
}

}  // namespace mav_state_estimation
