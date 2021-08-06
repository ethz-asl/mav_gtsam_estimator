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

#include "mav_state_estimation/initialization.h"

#include <eigen_conversions/eigen_msg.h>
#include <ros/assert.h>
#include <ros/console.h>
#define RAD2DEG M_PI/360.0*2.0
#define DEG2RAD 360.0/M_PI/2.0

namespace mav_state_estimation {

    void Initialization::addOrientationConstraint1(const Eigen::Vector3d &r_I,
                                                   const Eigen::Vector3d &r_B,
                                                   const ros::Time &t) {
        r_1 = std::make_optional<VectorPair>(r_I, r_B);
        stamp_ = t;
        computeInitialState();
    }

    void Initialization::addOrientationConstraint2(const Eigen::Vector3d &r_I,
                                                   const Eigen::Vector3d &r_B,
                                                   const ros::Time &t) {
        r_2 = std::make_optional<VectorPair>(r_I, r_B);
        stamp_ = t;
        computeInitialState();
    }

    void Initialization::addPositionConstraint(const Eigen::Vector3d &I_t_P,
                                               const Eigen::Vector3d &B_t_P,
                                               const ros::Time &t) {
        p = std::make_optional<VectorPair>(I_t_P, B_t_P);
        stamp_ = t;
        computeInitialState();
    }

    bool Initialization::getInitialPose(
            geometry_msgs::TransformStamped *T_IB) const {
        bool is_initialized = isInitialized();
        if (is_initialized) {
            *T_IB = *T_IB_;

        }
        return is_initialized;
    }

    void Initialization::computeInitialState() {
        bool is_initialized = r_1 && r_2 && p && inertial_frame_ && base_frame_;
        if (is_initialized) {
            Eigen::Matrix3d R_IB;
            calculateTriadOrientation(r_1->first, r_1->second, r_2->first, r_2->second,
                                      &R_IB);

            Eigen::Quaterniond q_IB(R_IB);
            ROS_WARN_STREAM("q_ib: " << q_IB.x() << " "<< q_IB.y() << " "<< q_IB.z() << " "<< q_IB.w());
            // output some debug info
            auto eulers = R_IB.eulerAngles(2, 1, 0);
            ROS_WARN_STREAM("Roll " << eulers[2]*RAD2DEG);
            ROS_WARN_STREAM("Pitch " << eulers[1]*RAD2DEG);
            ROS_WARN_STREAM("Yaw " << eulers[0]*RAD2DEG);

            Eigen::Vector3d I_t_B = p->first - R_IB * p->second;
            T_IB_ = std::make_optional<geometry_msgs::TransformStamped>();
            T_IB_->header.stamp = stamp_;
            T_IB_->header.frame_id = *inertial_frame_;
            T_IB_->child_frame_id = *base_frame_;
            tf::vectorEigenToMsg(I_t_B, T_IB_->transform.translation);
            tf::quaternionEigenToMsg(Eigen::Quaterniond(R_IB),
                                     T_IB_->transform.rotation);
        } else {
            ROS_DEBUG("Missing initial conditions.");
        }
    }

    void Initialization::calculateTriadOrientation(const Eigen::Vector3d &R1,
                                                   const Eigen::Vector3d &r1,
                                                   const Eigen::Vector3d &R2,
                                                   const Eigen::Vector3d &r2,
                                                   Eigen::Matrix3d *R_IB) const {
        ROS_ASSERT(R_IB);

        // Calculate the rotation between coordinate frames given two linearly
        // independent measurements. https://en.wikipedia.org/wiki/Triad_method
        Eigen::Vector3d S = R1.normalized();
        Eigen::Vector3d s = r1.normalized();
        Eigen::Vector3d M = (R1.cross(R2)).normalized();
        Eigen::Vector3d m = (r1.cross(r2)).normalized();

        Eigen::Matrix3d a;
        a << s, m, s.cross(m);
        Eigen::Matrix3d b;
        b << S, M, S.cross(M);
        *R_IB = b * a.transpose();
    }

}  // namespace mav_state_estimation
