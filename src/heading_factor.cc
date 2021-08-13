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
            gtsam::Key T_I_B_key, const double hdg_measured_ned_,
            const Eigen::Vector3d &B_t_PA,
            const gtsam::noiseModel::Base::shared_ptr &noise_model)
            : Base(noise_model, T_I_B_key),

            //calculate ENU heading and add offset based on calibration
              hdg_measured_enu_(HeadingAngle::fromNED(hdg_measured_ned_)),
              B_t_PA_(B_t_PA) {
        assert(noise_model);
    }

    gtsam::Vector HeadingFactor1::evaluateError(
            const gtsam::Pose3 &T_I_B, boost::optional<gtsam::Matrix &> D_Tt_T) const {

        // Calculate apparent heading angle of antenna configurations
        gtsam::Matrix33 D_Rt_R;
        auto I_t_P_A = T_I_B.rotation().rotate(B_t_PA_, D_Rt_R);
        HeadingAngle hdg_estimated_enu = HeadingAngle::fromENUVector(I_t_P_A);

        // get error (headingAngle class takes care of wraparound etc)
        HeadingAngle hdg_error_enu = hdg_estimated_enu - hdg_measured_enu_;

        // provide jacobian if needed
        if (D_Tt_T) {
            // Dimension: 1x6?
            D_Tt_T->resize(1, 6);
            D_Tt_T->setZero();
            D_Tt_T->matrix()(0, 2) = 1.0;   //we only affect yaw
        }

        /*
        ROS_WARN_STREAM(
                "Hdg Meas/Meas body/ Est/Err: " << hdg_measured_enu_ << "\t"
                                                << hdg_measured_enu_body << "\t"
                                                << hdg_estimated_enu << "\t"
                                                << hdg_error_enu);
        */
        Eigen::VectorXd result(1);
        result[0] = hdg_error_enu;
        return result;
    }

    void HeadingFactor1::print(const std::string &text,
                               const gtsam::KeyFormatter &key_formatter) const {
        std::cout << text << "MovingBaselineFactor1(" << key_formatter(this->key())
                  << ")\n";
        std::cout << "  measured enu heading: " << hdg_measured_enu_;
        std::cout << "  extrinsic calibration B_t_PA: " << B_t_PA_.transpose();
        this->noiseModel_->print("  noise model: ");
    }

    bool HeadingFactor1::equals(const gtsam::NonlinearFactor &expected,
                                double tolerance) const {
        const HeadingFactor1 *expected_casted =
                dynamic_cast<const HeadingFactor1 *>(&expected);  // NOLINT
        if (!expected_casted) return false;
        bool measured_equal =
                (hdg_measured_enu_ == expected_casted->hdg_measured_enu_);
        bool calibration_equal = (B_t_PA_ == expected_casted->B_t_PA_);
        return Base::equals(*expected_casted, tolerance) && measured_equal &&
               calibration_equal;
    }

}  // namespace mav_state_estimation
