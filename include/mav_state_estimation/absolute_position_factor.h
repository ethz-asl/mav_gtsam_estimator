#ifndef MAV_STATE_ESTIMATION_ABSOLUTE_POSITION_FACTOR_H_
#define MAV_STATE_ESTIMATION_ABSOLUTE_POSITION_FACTOR_H_

#include <string>

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/none.hpp>
#include <boost/shared_ptr.hpp>

namespace mav_state_estimation {

// Absolute position factor that determines the error and Jacobian w.r.t. a
// measured position in mission coordinates.
class AbsolutePositionFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 public:
  // Factor that determines the error and jacobian w.r.t. a measured position
  // in inertial coordinates.
  AbsolutePositionFactor(
      gtsam::Key T_I_B_key, gtsam::Key B_t_P_key,
      const gtsam::Point3& I_t_P_measured,
      const gtsam::noiseModel::Base::shared_ptr& noise_model);

  // Evaluates the error term and corresponding jacobians w.r.t. the pose.
  gtsam::Vector evaluateError(
      const gtsam::Pose3& T_I_B, const gtsam::Point3& B_t_P,
      boost::optional<gtsam::Matrix&> D_Tt_T = boost::none,
      boost::optional<gtsam::Matrix&> D_Tt_t = boost::none) const override;

  // Returns a deep copy of the factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  // Prints out information about the factor.
  void print(const std::string& s,
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  // Equality operator.
  bool equals(const gtsam::NonlinearFactor& expected,
              double tolerance) const override;

  // Returns the measured absolute position.
  inline const gtsam::Point3& measured() const { return I_t_P_measured_; }

  // Factory method.
  inline static shared_ptr Create(
      gtsam::Key T_I_B_key, gtsam::Key B_t_P_key,
      const gtsam::Point3& I_t_P_measured,
      const gtsam::noiseModel::Base::shared_ptr& noise_model) {
    return shared_ptr(new AbsolutePositionFactor(T_I_B_key, B_t_P_key,
                                                 I_t_P_measured, noise_model));
  }

 protected:
  // Absolute position measurement in local mission coordinates.
  const gtsam::Point3 I_t_P_measured_;

 private:
  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> Base;
  typedef AbsolutePositionFactor This;
};

}  // namespace mav_state_estimation

#endif  // MAV_STATE_ESTIMATION_ABSOLUTE_POSITION_FACTOR_H_
