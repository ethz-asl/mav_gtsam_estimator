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

#ifndef MAV_STATE_ESTIMATION_HEADING_FACTOR_H
#define MAV_STATE_ESTIMATION_HEADING_FACTOR_H

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Core>
#include <boost/none.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

namespace mav_state_estimation {

// Moving baseline factor that determines the error and Jacobian w.r.t.
// the heaading between two GPS antennas
class HeadingFactor1 : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  HeadingFactor1(gtsam::Key T_I_B_key,
                        const double I_t_PA_heading_measured,
                        const Eigen::Vector3d& B_t_PA,
                        const gtsam::noiseModel::Base::shared_ptr& noise_model);

  // Evaluates the error term and corresponding jacobians w.r.t. the pose.
  gtsam::Vector evaluateError(
      const gtsam::Pose3& T_I_B,
      boost::optional<gtsam::Matrix&> D_Tt_T = boost::none) const override;

  // Returns a deep copy of the factor.
  inline gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new HeadingFactor1(*this));
  }

  // Prints out information about the factor.
  void print(const std::string& s,
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  // Equality operator.
  bool equals(const gtsam::NonlinearFactor& expected,
              double tolerance) const override;

  // Returns the measured moving baseline.
  inline const double& measured() const { return I_t_PA_heading_measured_; }

  // Factory method.
  inline static shared_ptr Create(
      gtsam::Key T_I_B_key, const double I_t_PA_heading_measured,
      const Eigen::Vector3d& B_t_PA,
      const gtsam::noiseModel::Base::shared_ptr& noise_model) {
    return shared_ptr(new HeadingFactor1(T_I_B_key, I_t_PA_heading_measured,
                                                B_t_PA, noise_model));
  }

 protected:
  // Moving baseline measurement in inertial frame coordinates.
  const double I_t_PA_heading_measured_;
  // Extrinsic calibration from reference receiver antenna to attitude receiver
  // antenna in base frame.
  const Eigen::Vector3d B_t_PA_;

 private:
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
};

}  // namespace mav_state_estimation
#endif  // MAV_STATE_ESTIMATION_HEADING_FACTOR_H
