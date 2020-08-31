#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "mav_state_estimation/absolute_position_factor.h"

using namespace gtsam;
using namespace mav_state_estimation;
using symbol_shorthand::X;

TEST(AbsolutePositionFactor, Error) {
  Rot3 R = Rot3::rodriguez(0.0, 0.0, 0.0);
  Point3 t(1.0, 0.5, 0.2);
  Pose3 pose(R, t);
  Point3 I_t_P(11.0, 0.5, 0.2);
  Point3 B_t_P_(10.0, 0.0, 0.0);
  Point3 I_t_P_measured = I_t_P;

  AbsolutePositionFactor factor(X(1), I_t_P_measured, B_t_P_,
                                noiseModel::Isotropic::Sigma(3, 0.05));
  Vector expected_error = (Vector(3) << 0.0, 0.0, 0.0).finished();
  Vector actual_error = factor.evaluateError(pose);
  EXPECT_TRUE(assert_equal(expected_error, actual_error));
}

TEST(AbsolutePositionFactor, Jacobian) {
  Rot3 R = Rot3::rodriguez(0.1, 0.2, 0.3);
  Point3 t(1.0, 0.5, 0.2);
  Pose3 pose(R, t);
  Point3 I_t_P(11.0, 0.5, 0.2);
  Point3 B_t_P_(10.0, 0.0, 0.0);
  Point3 I_t_P_measured = I_t_P;

  AbsolutePositionFactor factor(X(1), I_t_P_measured, B_t_P_,
                                noiseModel::Isotropic::Sigma(3, 0.05));

  Matrix actual_D_Tt_T;
  factor.evaluateError(pose, actual_D_Tt_T);

  Matrix numerical_D_Tt_T = numericalDerivative11(
      boost::function<Vector(const Pose3&)>(boost::bind(
          &AbsolutePositionFactor::evaluateError, factor, _1, boost::none)),
      pose, 1e-5);
  EXPECT_TRUE(assert_equal(numerical_D_Tt_T, actual_D_Tt_T, 1e-5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
