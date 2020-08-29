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
class MovingBaselineFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  MovingBaselineFactor(gtsam::Key T_I_B_key,
                       const Eigen::Vector3d& I_t_PA_measured,
                       const Eigen::Vector3d& B_t_PA,
                       const gtsam::noiseModel::Base::shared_ptr& noise_model);

  // Evaluates the error term and corresponding jacobians w.r.t. the pose.
  gtsam::Vector evaluateError(const gtsam::Pose3& T_I_B,
                              boost::optional<gtsam::Matrix&> J_err_wrt_T_I_B =
                                  boost::none) const override;

  // Returns a deep copy of the factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  // Prints out information about the factor.
  void print(const std::string& s,
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  // Equality operator.
  bool equals(const gtsam::NonlinearFactor& expected,
              double tolerance) const override;

  // Returns the measured moving baseline.
  inline const Eigen::Vector3d& measured() const { return I_t_PA_measured_; }

  // Returns the number of variables attached to this factor. This is a unary
  // factor.
  inline size_t size() const { return 1u; }

  // Returns the dimension of the measurement.
  inline size_t dim() const { return 3u; }

  // Factory method.
  inline static shared_ptr Create(
      gtsam::Key T_I_B_key, const Eigen::Vector3d& I_t_PA_measured,
      const Eigen::Vector3d& B_t_PA,
      const gtsam::noiseModel::Base::shared_ptr& noise_model) {
    return shared_ptr(new MovingBaselineFactor(T_I_B_key, I_t_PA_measured,
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
  typedef MovingBaselineFactor This;
};

}  // namespace mav_state_estimation

#endif  // MAV_STATE_ESTIMATION_MOVING_BASELINE_FACTOR_H_
