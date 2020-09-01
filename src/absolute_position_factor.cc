#include "mav_state_estimation/absolute_position_factor.h"

#include <gtsam/base/OptionalJacobian.h>
#include <ros/ros.h>

namespace mav_state_estimation {

AbsolutePositionFactor1::AbsolutePositionFactor1(
    gtsam::Key T_I_B_key, const gtsam::Point3& I_t_P_measured,
    const gtsam::Point3& B_t_P,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : Base(noise_model, T_I_B_key),
      I_t_P_measured_(I_t_P_measured),
      B_t_P_(B_t_P) {
  assert(noise_model);
}

gtsam::Vector AbsolutePositionFactor1::evaluateError(
    const gtsam::Pose3& T_I_B, boost::optional<gtsam::Matrix&> D_Tt_T) const {
  // h(R,p) = I_t_B + R_I_B * B_t_P
  gtsam::Vector3 h;
  if (D_Tt_T) {
    D_Tt_T->resize(3, 6);
    h = T_I_B.transform_from(B_t_P_, D_Tt_T);
  } else {
    h = T_I_B.transform_from(B_t_P_);
  }

  // error = h - z
  return h - I_t_P_measured_;
}

void AbsolutePositionFactor1::print(
    const std::string& text, const gtsam::KeyFormatter& key_formatter) const {
  std::cout << text << "AbsolutePositionFactor1(" << key_formatter(this->key())
            << ")\n";
  std::cout << "  measured: " << I_t_P_measured_.transpose();
  std::cout << "  extrinsic calibration: " << B_t_P_.transpose();
  this->noiseModel_->print("  noise model: ");
}

bool AbsolutePositionFactor1::equals(const gtsam::NonlinearFactor& expected,
                                     double tolerance) const {
  const This* expected_casted = dynamic_cast<const This*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool measured_equal = (I_t_P_measured_ == expected_casted->I_t_P_measured_);
  bool calibration_equal = (B_t_P_ == expected_casted->B_t_P_);
  return Base::equals(*expected_casted, tolerance) && measured_equal &&
         calibration_equal;
}

AbsolutePositionFactor2::AbsolutePositionFactor2(
    gtsam::Key T_I_B_key, gtsam::Key B_t_P_key,
    const gtsam::Point3& I_t_P_measured,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : Base(noise_model, T_I_B_key, B_t_P_key), I_t_P_measured_(I_t_P_measured) {
  assert(noise_model);
}

gtsam::Vector AbsolutePositionFactor2::evaluateError(
    const gtsam::Pose3& T_I_B, const gtsam::Point3& B_t_P,
    boost::optional<gtsam::Matrix&> D_Tt_T,
    boost::optional<gtsam::Matrix&> D_Tt_t) const {
  // h(R,p) = I_t_B + R_I_B * B_t_P
  gtsam::Vector3 h;
  if (D_Tt_T || D_Tt_t) {
    gtsam::Matrix36 D_Tt_T_tmp;
    gtsam::Matrix33 D_Tt_t_tmp;
    h = T_I_B.transform_from(B_t_P, D_Tt_T_tmp, D_Tt_t_tmp);
    if (D_Tt_T) {
      D_Tt_T->resize(3, 6);
      *D_Tt_T = D_Tt_T_tmp;
    }
    if (D_Tt_t) {
      D_Tt_t->resize(3, 3);
      *D_Tt_t = D_Tt_t_tmp;
    }
  } else {
    h = T_I_B.transform_from(B_t_P);
  }

  // error = h - z
  return h - I_t_P_measured_;
}

void AbsolutePositionFactor2::print(
    const std::string& text, const gtsam::KeyFormatter& key_formatter) const {
  std::cout << text << "AbsolutePositionFactor2(" << key_formatter(this->key1())
            << ", " << key_formatter(this->key2()) << ")\n";
  std::cout << "  measured: " << I_t_P_measured_.transpose();
  this->noiseModel_->print("  noise model: ");
}

bool AbsolutePositionFactor2::equals(const gtsam::NonlinearFactor& expected,
                                     double tolerance) const {
  const This* expected_casted = dynamic_cast<const This*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool measured_equal = (I_t_P_measured_ == expected_casted->I_t_P_measured_);
  return Base::equals(*expected_casted, tolerance) && measured_equal;
}

}  // namespace mav_state_estimation
