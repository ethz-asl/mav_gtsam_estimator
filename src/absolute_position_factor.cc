#include "mav_state_estimation/absolute_position_factor.h"

#include <gtsam/base/OptionalJacobian.h>

namespace mav_state_estimation {

AbsolutePositionFactor::AbsolutePositionFactor(
    gtsam::Key T_I_B_key, const gtsam::Point3& I_t_P_measured,
    const gtsam::Point3& B_t_P,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : Base(noise_model, T_I_B_key),
      I_t_P_measured_(I_t_P_measured),
      B_t_P_(B_t_P) {
  assert(noise_model);
}

gtsam::Vector AbsolutePositionFactor::evaluateError(
    const gtsam::Pose3& T_I_B, boost::optional<gtsam::Matrix&> H) const {
  // h(R,p) = I_t_B + R_I_B * B_t_P
  gtsam::Vector3 h;
  if (H) {
    H->resize(3, 6);
    h = T_I_B.transform_from(B_t_P_, H);
  } else {
    h = T_I_B.transform_from(B_t_P_);
  }

  // error = h - z
  return h - I_t_P_measured_;
}

gtsam::NonlinearFactor::shared_ptr AbsolutePositionFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new This(*this));
}

void AbsolutePositionFactor::print(
    const std::string& text, const gtsam::KeyFormatter& key_formatter) const {
  std::cout << text << "AbsolutePositionFactor(" << key_formatter(this->key())
            << ")\n";
  std::cout << "  measured: " << I_t_P_measured_.transpose();
  std::cout << "  extrinsic calibration: " << B_t_P_.transpose();
  this->noiseModel_->print("  noise model: ");
}

bool AbsolutePositionFactor::equals(const gtsam::NonlinearFactor& expected,
                                    double tolerance) const {
  const This* expected_casted = dynamic_cast<const This*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool measured_equal = (I_t_P_measured_ == expected_casted->I_t_P_measured_);
  bool calibration_equal = (B_t_P_ == expected_casted->B_t_P_);
  return Base::equals(*expected_casted, tolerance) && measured_equal &&
         calibration_equal;
}

}  // namespace mav_state_estimation
