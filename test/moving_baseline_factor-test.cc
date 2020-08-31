#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "mav_state_estimation/moving_baseline_factor.h"

using namespace gtsam;
using namespace mav_state_estimation;
using symbol_shorthand::A;
using symbol_shorthand::P;
using symbol_shorthand::X;

TEST(MovingBaselineFactor, Error) {
  Rot3 R = Rot3::Yaw(M_PI / 2);
  Point3 t(1.0, 0.5, 0.2);
  Pose3 pose(R, t);
  Point3 B_t_P(10.0, 0.0, 0.0);
  Point3 B_t_A(11.0, 0.0, 0.0);
  Point3 I_t_PA_measured(0.0, 1.0, 0.0);

  MovingBaselineFactor factor(X(1), P(1), A(1), I_t_PA_measured,
                              noiseModel::Isotropic::Sigma(3, 0.05));
  Vector expected_error = (Vector(3) << 0.0, 0.0, 0.0).finished();
  Vector actual_error = factor.evaluateError(pose, B_t_P, B_t_A);
  EXPECT_TRUE(assert_equal(expected_error, actual_error, 1e-5));
}

TEST(MovingBaselineFactor, Jacobian) {
  Rot3 R = Rot3::rodriguez(0.1, 0.2, 0.3);
  Point3 t(1.0, 0.5, 0.2);
  Pose3 pose(R, t);
  Point3 B_t_P(10.0, 0.0, 0.0);
  Point3 B_t_A(11.0, 0.0, 0.0);
  Point3 I_t_PA_measured(0.0, 1.0, 0.0);

  MovingBaselineFactor factor(X(1), P(1), A(1), I_t_PA_measured,
                              noiseModel::Isotropic::Sigma(3, 0.05));
  EXPECT_EQ(3u, factor.dim());

  Matrix actual_D_Tt_T, actual_D_Tt_tP, actual_D_Tt_tA;
  factor.evaluateError(pose, B_t_P, B_t_A, actual_D_Tt_T, actual_D_Tt_tP,
                       actual_D_Tt_tA);

  Matrix numerical_D_Tt_T = numericalDerivative31(
      boost::function<Vector(const Pose3&, const Point3&, const Point3&)>(
          boost::bind(&MovingBaselineFactor::evaluateError, factor, _1, _2, _3,
                      boost::none, boost::none, boost::none)),
      pose, B_t_P, B_t_A, 1e-5);
  EXPECT_TRUE(assert_equal(numerical_D_Tt_T, actual_D_Tt_T, 1e-5));

  Matrix numerical_D_Tt_tP = numericalDerivative32(
      boost::function<Vector(const Pose3&, const Point3&, const Point3&)>(
          boost::bind(&MovingBaselineFactor::evaluateError, factor, _1, _2, _3,
                      boost::none, boost::none, boost::none)),
      pose, B_t_P, B_t_A, 1e-5);
  EXPECT_TRUE(assert_equal(numerical_D_Tt_tP, actual_D_Tt_tP, 1e-5));

  Matrix numerical_D_Tt_tA = numericalDerivative33(
      boost::function<Vector(const Pose3&, const Point3&, const Point3&)>(
          boost::bind(&MovingBaselineFactor::evaluateError, factor, _1, _2, _3,
                      boost::none, boost::none, boost::none)),
      pose, B_t_P, B_t_A, 1e-5);
  EXPECT_TRUE(assert_equal(numerical_D_Tt_tA, actual_D_Tt_tA, 1e-5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
