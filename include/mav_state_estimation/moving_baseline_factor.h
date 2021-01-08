/*
MIT License
Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland
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

#ifndef MAV_STATE_ESTIMATION_MOVING_BASELINE_FACTOR_H_
#define MAV_STATE_ESTIMATION_MOVING_BASELINE_FACTOR_H_

#include <string>

#include <Eigen/Core>

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/none.hpp>
#include <boost/shared_ptr.hpp>

namespace mav_state_estimation {

// Moving baseline factor that determines the error and Jacobian w.r.t. to the
// vector between two GNSS antennas in inertial frame.
class MovingBaselineFactor1 : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  MovingBaselineFactor1(gtsam::Key T_I_B_key,
                        const Eigen::Vector3d& I_t_PA_measured,
                        const Eigen::Vector3d& B_t_PA,
                        const gtsam::noiseModel::Base::shared_ptr& noise_model);

  // Evaluates the error term and corresponding jacobians w.r.t. the pose.
  gtsam::Vector evaluateError(
      const gtsam::Pose3& T_I_B,
      boost::optional<gtsam::Matrix&> D_Tt_T = boost::none) const override;

  // Returns a deep copy of the factor.
  inline gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new This(*this));
  }

  // Prints out information about the factor.
  void print(const std::string& s,
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  // Equality operator.
  bool equals(const gtsam::NonlinearFactor& expected,
              double tolerance) const override;

  // Returns the measured moving baseline.
  inline const Eigen::Vector3d& measured() const { return I_t_PA_measured_; }

  // Factory method.
  inline static shared_ptr Create(
      gtsam::Key T_I_B_key, const Eigen::Vector3d& I_t_PA_measured,
      const Eigen::Vector3d& B_t_PA,
      const gtsam::noiseModel::Base::shared_ptr& noise_model) {
    return shared_ptr(new MovingBaselineFactor1(T_I_B_key, I_t_PA_measured,
                                                B_t_PA, noise_model));
  }

 protected:
  // Moving baseline measurement in inertial frame coordinates.
  const Eigen::Vector3d I_t_PA_measured_;
  // Extrinsic calibration from reference receiver antenna to attitude receiver
  // antenna in base frame.
  const Eigen::Vector3d B_t_PA_;

 private:
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
  typedef MovingBaselineFactor1 This;
};

// Moving baseline factor that determines the error and Jacobian w.r.t. to the
// vector between two GNSS antennas in inertial frame. Additionally estimates
// position antenna calibrations.
class MovingBaselineFactor2
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Point3,
                                      gtsam::Point3> {
 public:
  MovingBaselineFactor2(gtsam::Key T_I_B_key, gtsam::Key B_t_P_key,
                        gtsam::Key B_t_A_key,
                        const Eigen::Vector3d& I_t_PA_measured,
                        const gtsam::noiseModel::Base::shared_ptr& noise_model);

  // Evaluates the error term and corresponding jacobians w.r.t. the pose.
  gtsam::Vector evaluateError(
      const gtsam::Pose3& T_I_B, const gtsam::Point3& B_t_P,
      const gtsam::Point3& B_t_A,
      boost::optional<gtsam::Matrix&> D_Tt_T = boost::none,
      boost::optional<gtsam::Matrix&> D_Tt_tP = boost::none,
      boost::optional<gtsam::Matrix&> D_Tt_tA = boost::none) const override;

  // Returns a deep copy of the factor.
  inline gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new This(*this));
  }

  // Prints out information about the factor.
  void print(const std::string& s,
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  // Equality operator.
  bool equals(const gtsam::NonlinearFactor& expected,
              double tolerance) const override;

  // Returns the measured moving baseline.
  inline const Eigen::Vector3d& measured() const { return I_t_PA_measured_; }

  // Factory method.
  inline static shared_ptr Create(
      gtsam::Key T_I_B_key, gtsam::Key B_t_P_key, gtsam::Key B_t_A_key,
      const Eigen::Vector3d& I_t_PA_measured,
      const gtsam::noiseModel::Base::shared_ptr& noise_model) {
    return shared_ptr(new MovingBaselineFactor2(T_I_B_key, B_t_P_key, B_t_A_key,
                                                I_t_PA_measured, noise_model));
  }

 protected:
  // Moving baseline measurement in inertial frame coordinates.
  const Eigen::Vector3d I_t_PA_measured_;

 private:
  typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Point3, gtsam::Point3>
      Base;
  typedef MovingBaselineFactor2 This;
};

}  // namespace mav_state_estimation

#endif  // MAV_STATE_ESTIMATION_MOVING_BASELINE_FACTOR_H_
