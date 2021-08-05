/*
MIT License
Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland
Copyright (c) 2021 Michael Pantic, ASL, ETH Zurich, Switzerland
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "mav_state_estimation/heading_factor.h"

#include <gtsam/base/OptionalJacobian.h>
#include <ros/ros.h>

namespace mav_state_estimation {

HeadingFactor1::HeadingFactor1(
    gtsam::Key T_I_B_key, const double I_t_PA_heading_measured_,
    const Eigen::Vector3d& B_t_PA,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : Base(noise_model, T_I_B_key),
      I_t_PA_heading_measured_(I_t_PA_heading_measured_),
      B_t_PA_(B_t_PA) {
  assert(noise_model);
}

gtsam::Vector HeadingFactor1::evaluateError(
    const gtsam::Pose3& T_I_B, boost::optional<gtsam::Matrix&> D_Tt_T) const {
  // the math of this might be a bit shaky - just a first test.

  // get unit vector in ENU (will be in E-N plane always)
  Eigen::Matrix3d r_I_B_yaw_measured(
      Eigen::AngleAxisd(I_t_PA_heading_measured_, Eigen::Vector3d::UnitZ())
      );

  // the yaw we obtain is in NED, so we need to rotate the North vector (Y) in ENU.
  Eigen::Vector3d hdg_measured = r_I_B_yaw_measured * Eigen::Vector3d::UnitY();

  // decouple Roll pitch from yaw
  // only valid because we don't fly omnidirectionally!!
  Eigen::Matrix3d r_I_B = T_I_B.rotation().matrix();
  double hdg = r_I_B.eulerAngles(2, 1, 0).x();
  Eigen::Matrix3d r_I_B_yaw(Eigen::AngleAxisd(hdg, Eigen::Vector3d::UnitZ()));

  gtsam::Pose3 T_I_B_yaw(gtsam::Rot3(r_I_B_yaw), T_I_B.translation());

  // h(R,p) = R_IB_yaw * (B_t_PA).normalized()
  // important, we compared against the normalized vector!
  Eigen::Vector3d h;
  if (D_Tt_T) {
    gtsam::Matrix33 D_Rt_R;
    h = T_I_B_yaw.rotation().rotate(B_t_PA_.normalized(), D_Rt_R);
    D_Tt_T->resize(3, 6);
    D_Tt_T->leftCols<3>() = D_Rt_R;
    D_Tt_T->rightCols<3>() = gtsam::Matrix::Zero(3, 3);
  } else {
    h = T_I_B_yaw.rotation().rotate(B_t_PA_.normalized());
  }

  // error = h - z
  return h - hdg_measured;
}

void HeadingFactor1::print(const std::string& text,
                           const gtsam::KeyFormatter& key_formatter) const {
  std::cout << text << "MovingBaselineFactor1(" << key_formatter(this->key())
            << ")\n";
  std::cout << "  measured I_t_PA heading: " << I_t_PA_heading_measured_;
  std::cout << "  extrinsic calibration B_t_PA: " << B_t_PA_.transpose();
  this->noiseModel_->print("  noise model: ");
}

bool HeadingFactor1::equals(const gtsam::NonlinearFactor& expected,
                            double tolerance) const {
  const HeadingFactor1* expected_casted =
      dynamic_cast<const HeadingFactor1*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool measured_equal =
      (I_t_PA_heading_measured_ == expected_casted->I_t_PA_heading_measured_);
  bool calibration_equal = (B_t_PA_ == expected_casted->B_t_PA_);
  return Base::equals(*expected_casted, tolerance) && measured_equal &&
         calibration_equal;
}

}  // namespace mav_state_estimation
