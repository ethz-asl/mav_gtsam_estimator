#include "mav_state_estimation/moving_baseline_factor.h"

#include <gtsam/base/OptionalJacobian.h>
#include <ros/ros.h>

namespace mav_state_estimation {

MovingBaselineFactor1::MovingBaselineFactor1(
    gtsam::Key T_I_B_key, const Eigen::Vector3d& I_t_PA_measured,
    const Eigen::Vector3d& B_t_PA,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : Base(noise_model, T_I_B_key),
      I_t_PA_measured_(I_t_PA_measured),
      B_t_PA_(B_t_PA) {
  assert(noise_model);
}

gtsam::Vector MovingBaselineFactor1::evaluateError(
    const gtsam::Pose3& T_I_B, boost::optional<gtsam::Matrix&> D_Tt_T) const {
  // h(R,p) = R_IB * (B_t_PA)
  Eigen::Vector3d h;
  if (D_Tt_T) {
    gtsam::Matrix33 D_Rt_R;
    h = T_I_B.rotation().rotate(B_t_PA_, D_Rt_R);
    D_Tt_T->resize(3, 6);
    D_Tt_T->leftCols<3>() = D_Rt_R;
    D_Tt_T->rightCols<3>() = gtsam::Matrix::Zero(3, 3);
  } else {
    h = T_I_B.rotation().rotate(B_t_PA_);
  }

  // error = h - z
  return h - I_t_PA_measured_;
}

void MovingBaselineFactor1::print(
    const std::string& text, const gtsam::KeyFormatter& key_formatter) const {
  std::cout << text << "MovingBaselineFactor1(" << key_formatter(this->key())
            << ")\n";
  std::cout << "  measured I_t_PA: " << I_t_PA_measured_.transpose();
  std::cout << "  extrinsic calibration B_t_PA: " << B_t_PA_.transpose();
  this->noiseModel_->print("  noise model: ");
}

bool MovingBaselineFactor1::equals(const gtsam::NonlinearFactor& expected,
                                   double tolerance) const {
  const This* expected_casted = dynamic_cast<const This*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool measured_equal = (I_t_PA_measured_ == expected_casted->I_t_PA_measured_);
  bool calibration_equal = (B_t_PA_ == expected_casted->B_t_PA_);
  return Base::equals(*expected_casted, tolerance) && measured_equal &&
         calibration_equal;
}

MovingBaselineFactor2::MovingBaselineFactor2(
    gtsam::Key T_I_B_key, gtsam::Key B_t_P_key, gtsam::Key B_t_A_key,
    const Eigen::Vector3d& I_t_PA_measured,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : Base(noise_model, T_I_B_key, B_t_P_key, B_t_A_key),
      I_t_PA_measured_(I_t_PA_measured) {
  assert(noise_model);
}

gtsam::Vector MovingBaselineFactor2::evaluateError(
    const gtsam::Pose3& T_I_B, const gtsam::Point3& B_t_P,
    const gtsam::Point3& B_t_A, boost::optional<gtsam::Matrix&> D_Tt_T,
    boost::optional<gtsam::Matrix&> D_Tt_tP,
    boost::optional<gtsam::Matrix&> D_Tt_tA) const {
  // h(R,p) = R_IB * (B_t_A - B_t_P)
  Eigen::Vector3d h;
  auto B_t_PA = B_t_A - B_t_P;
  if (D_Tt_T || D_Tt_tP || D_Tt_tA) {
    gtsam::Matrix33 D_Rt_R, D_Rt_tA;
    h = T_I_B.rotation().rotate(B_t_PA, D_Rt_R, D_Rt_tA);
    if (D_Tt_T) {
      D_Tt_T->resize(3, 6);
      D_Tt_T->leftCols<3>() = D_Rt_R;
      D_Tt_T->rightCols<3>() = gtsam::Matrix::Zero(3, 3);
    }
    if (D_Tt_tP) {
      D_Tt_tP->resize(3, 3);
      // The partial derivative D_Rt_tA is matrix R*I, thus this one is -R*I.
      *D_Tt_tP = -D_Rt_tA;
    }
    if (D_Tt_tA) {
      D_Tt_tA->resize(3, 3);
      *D_Tt_tA = D_Rt_tA;
    }
  } else {
    h = T_I_B.rotation().rotate(B_t_PA);
  }

  // error = h - z
  return h - I_t_PA_measured_;
}

void MovingBaselineFactor2::print(
    const std::string& text, const gtsam::KeyFormatter& key_formatter) const {
  std::cout << text << "MovingBaselineFactor2(" << key_formatter(this->key1())
            << ", " << key_formatter(this->key2()) << ", "
            << key_formatter(this->key3()) << ")\n";
  std::cout << "  measured I_t_PA: " << I_t_PA_measured_.transpose();
  this->noiseModel_->print("  noise model: ");
}

bool MovingBaselineFactor2::equals(const gtsam::NonlinearFactor& expected,
                                   double tolerance) const {
  const This* expected_casted = dynamic_cast<const This*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool measured_equal = (I_t_PA_measured_ == expected_casted->I_t_PA_measured_);
  return Base::equals(*expected_casted, tolerance) && measured_equal;
}

}  // namespace mav_state_estimation
