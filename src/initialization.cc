#include "mav_state_estimation/initialization.h"

#include <eigen_conversions/eigen_msg.h>
#include <ros/assert.h>
#include <ros/console.h>

namespace mav_state_estimation {

void Initialization::addOrientationConstraint1(const Eigen::Vector3d& r_I,
                                               const Eigen::Vector3d& r_B,
                                               const ros::Time& t) {
  if (!isInitialized()) {
    r_1 = std::make_optional<VectorPair>(r_I, r_B);
    stamp_ = t;
    computeInitialState();
  }
}

void Initialization::addOrientationConstraint2(const Eigen::Vector3d& r_I,
                                               const Eigen::Vector3d& r_B,
                                               const ros::Time& t) {
  if (!isInitialized()) {
    r_2 = std::make_optional<VectorPair>(r_I, r_B);
    stamp_ = t;
    computeInitialState();
  }
}

void Initialization::addPositionConstraint(const Eigen::Vector3d& I_t_P,
                                           const Eigen::Vector3d& B_t_P,
                                           const ros::Time& t) {
  if (!isInitialized()) {
    p = std::make_optional<VectorPair>(I_t_P, B_t_P);
    stamp_ = t;
    computeInitialState();
  }
}

bool Initialization::getInitialPose(
    geometry_msgs::TransformStamped* T_IB) const {
  bool is_initialized = isInitialized();
  if (is_initialized) {
    *T_IB = *T_IB_;
  }
  return is_initialized;
}

void Initialization::computeInitialState() {
  bool is_initialized = r_1 && r_2 && p && inertial_frame_ && base_frame_;
  if (is_initialized) {
    Eigen::Matrix3d R_IB;
    calculateTriadOrientation(r_1->first, r_1->second, r_2->first, r_2->second,
                              &R_IB);
    Eigen::Vector3d I_t_B = p->first - R_IB * p->second;
    T_IB_ = std::make_optional<geometry_msgs::TransformStamped>();
    T_IB_->header.stamp = stamp_;
    T_IB_->header.frame_id = *inertial_frame_;
    T_IB_->child_frame_id = *base_frame_;
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
  Eigen::Matrix3d b;
  b << S, M, S.cross(M);
  *R_IB = b * a.transpose();
}

}  // namespace mav_state_estimation
