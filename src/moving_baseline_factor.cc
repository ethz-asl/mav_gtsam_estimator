#include "mav_state_estimation/moving_baseline_factor.h"

#include <gtsam/base/OptionalJacobian.h>

namespace mav_state_estimation {

MovingBaselineFactor::MovingBaselineFactor(
    gtsam::Key T_I_B_key, const Eigen::Vector3d& I_t_PA_measured,
    const Eigen::Vector3d& B_t_PA,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : Base(noise_model, T_I_B_key),
      I_t_PA_measured_(I_t_PA_measured),
      B_t_PA_(B_t_PA) {
  assert(noise_model);
}

gtsam::Vector MovingBaselineFactor::evaluateError(
    const gtsam::Pose3& T_I_B,
    boost::optional<gtsam::Matrix&> J_err_wrt_T_I_B) const {
  // h(R,p) = R_IB * (B_t_PA)
  Eigen::Vector3d h;
  if (J_err_wrt_T_I_B) {
    J_err_wrt_T_I_B->resize(3, 6);
    h = T_I_B.rotation().rotate(B_t_PA_, J_err_wrt_T_I_B);
  } else {
    h = T_I_B.rotation().rotate(B_t_PA_);
  }

  // error = h - z
  return h - I_t_PA_measured_;
}

gtsam::NonlinearFactor::shared_ptr MovingBaselineFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new This(*this));
}

void MovingBaselineFactor::print(
    const std::string& text, const gtsam::KeyFormatter& key_formatter) const {
  std::cout << text << "MovingBaselineFactor(" << key_formatter(this->key())
            << ")\n";
  std::cout << "  measured I_t_PA: " << I_t_PA_measured_.transpose();
  std::cout << "  extrinsic calibration B_t_PA: " << B_t_PA_.transpose();
  this->noiseModel_->print("  noise model: ");
}

bool MovingBaselineFactor::equals(const gtsam::NonlinearFactor& expected,
                                  double tolerance) const {
  const This* expected_casted = dynamic_cast<const This*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool measured_equal = (I_t_PA_measured_ == expected_casted->I_t_PA_measured_);
  bool calibration_equal = (B_t_PA_ == expected_casted->B_t_PA_);
  return Base::equals(*expected_casted, tolerance) && measured_equal &&
         calibration_equal;
}

}  // namespace mav_state_estimation
