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

TEST(MovingBaselineFactor2, Error) {
  Rot3 R = Rot3::Yaw(M_PI / 2);
  Point3 t(1.0, 0.5, 0.2);
  Pose3 pose(R, t);
  Point3 B_t_P(10.0, 0.0, 0.0);
  Point3 B_t_A(11.0, 0.0, 0.0);
  Point3 I_t_PA_measured(0.0, 1.0, 0.0);

  MovingBaselineFactor1 factor1(X(1), I_t_PA_measured, B_t_A - B_t_P,
                                noiseModel::Isotropic::Sigma(3, 0.05));
  MovingBaselineFactor2 factor2(X(1), P(1), A(1), I_t_PA_measured,
                                noiseModel::Isotropic::Sigma(3, 0.05));
  Vector expected_error = (Vector(3) << 0.0, 0.0, 0.0).finished();
  Vector actual_error1 = factor1.evaluateError(pose);
  Vector actual_error2 = factor2.evaluateError(pose, B_t_P, B_t_A);
  EXPECT_TRUE(assert_equal(expected_error, actual_error1, 1e-5));
  EXPECT_TRUE(assert_equal(expected_error, actual_error2, 1e-5));
}

TEST(MovingBaselineFactor2, Jacobian) {
  Rot3 R = Rot3::rodriguez(0.1, 0.2, 0.3);
  Point3 t(1.0, 0.5, 0.2);
  Pose3 pose(R, t);
  Point3 B_t_P(10.0, 0.0, 0.0);
  Point3 B_t_A(11.0, 0.0, 0.0);
  Point3 I_t_PA_measured(0.0, 1.0, 0.0);

  MovingBaselineFactor1 factor1(X(1), I_t_PA_measured, B_t_A - B_t_P,
                                noiseModel::Isotropic::Sigma(3, 0.05));
  MovingBaselineFactor2 factor2(X(1), P(1), A(1), I_t_PA_measured,
                                noiseModel::Isotropic::Sigma(3, 0.05));

  Matrix actual1_D_Tt_T;
  factor1.evaluateError(pose, actual1_D_Tt_T);

  Matrix numerical1_D_Tt_T = numericalDerivative11(
      boost::function<Vector(const Pose3&)>(boost::bind(
          &MovingBaselineFactor1::evaluateError, factor1, _1, boost::none)),
      pose, 1e-5);
  EXPECT_TRUE(assert_equal(numerical1_D_Tt_T, actual1_D_Tt_T, 1e-5));

  Matrix actual2_D_Tt_T, actual2_D_Tt_TP, actual2_D_Tt_TA;
  factor2.evaluateError(pose, B_t_P, B_t_A, actual2_D_Tt_T, actual2_D_Tt_TP,
                        actual2_D_Tt_TA);

  Matrix numerical2_D_Tt_T = numericalDerivative31(
      boost::function<Vector(const Pose3&, const Point3&, const Point3&)>(
          boost::bind(&MovingBaselineFactor2::evaluateError, factor2, _1, _2,
                      _3, boost::none, boost::none, boost::none)),
      pose, B_t_P, B_t_A, 1e-5);
  EXPECT_TRUE(assert_equal(numerical2_D_Tt_T, actual2_D_Tt_T, 1e-5));

  Matrix numerical2_D_Tt_TP = numericalDerivative32(
      boost::function<Vector(const Pose3&, const Point3&, const Point3&)>(
          boost::bind(&MovingBaselineFactor2::evaluateError, factor2, _1, _2,
                      _3, boost::none, boost::none, boost::none)),
      pose, B_t_P, B_t_A, 1e-5);
  EXPECT_TRUE(assert_equal(numerical2_D_Tt_TP, actual2_D_Tt_TP, 1e-5));

  Matrix numerical2_D_Tt_TA = numericalDerivative33(
      boost::function<Vector(const Pose3&, const Point3&, const Point3&)>(
          boost::bind(&MovingBaselineFactor2::evaluateError, factor2, _1, _2,
                      _3, boost::none, boost::none, boost::none)),
      pose, B_t_P, B_t_A, 1e-5);
  EXPECT_TRUE(assert_equal(numerical2_D_Tt_TA, actual2_D_Tt_TA, 1e-5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
