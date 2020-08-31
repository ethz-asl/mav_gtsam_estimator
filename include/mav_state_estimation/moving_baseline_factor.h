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
class MovingBaselineFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Point3,
                                      gtsam::Point3> {
 public:
  MovingBaselineFactor(gtsam::Key T_I_B_key, gtsam::Key B_t_P_key,
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

  // Factory method.
  inline static shared_ptr Create(
      gtsam::Key T_I_B_key, gtsam::Key B_t_P_key, gtsam::Key B_t_A_key,
      const Eigen::Vector3d& I_t_PA_measured,
      const gtsam::noiseModel::Base::shared_ptr& noise_model) {
    return shared_ptr(new MovingBaselineFactor(T_I_B_key, B_t_P_key, B_t_A_key,
                                               I_t_PA_measured, noise_model));
  }

 protected:
  // Moving baseline measurement in inertial frame coordinates.
  const Eigen::Vector3d I_t_PA_measured_;

 private:
  typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Point3, gtsam::Point3>
      Base;
  typedef MovingBaselineFactor This;
};

}  // namespace mav_state_estimation

#endif  // MAV_STATE_ESTIMATION_MOVING_BASELINE_FACTOR_H_