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

#include "mav_state_estimation/mav_state_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>

#include "mav_state_estimation/BatchStatus.h"
#include "mav_state_estimation/Timing.h"
#include "mav_state_estimation/absolute_position_factor.h"
#include "mav_state_estimation/heading_factor.h"
#include "mav_state_estimation/moving_baseline_factor.h"

using gtsam::symbol_shorthand::A;  // Attitude receiver antenna (x,y,z)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::P;  // Position receiver antenna (x,y,z)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dRow;

namespace mav_state_estimation {

    MavStateEstimator::MavStateEstimator(const ros::NodeHandle &nh,
                                         const ros::NodeHandle &nh_private)
            : nh_(nh), nh_private_(nh_private) {
        // Get parameters.
        nh_private_.getParam("external_poses", external_poses_);
        nh_private_.getParam("odometry_throttle", odometry_throttle_);

        // GNSS
        nh_private_.getParam("estimate_antenna_positions",
                             estimate_antenna_positions_);
        loadGnssParams("position_receiver", P(0), &B_t_P_,
                       &process_noise_model_B_t_P_, &pos_receiver_cov_scale_);
        loadGnssParams("attitude_receiver", A(0), &B_t_A_,
                       &process_noise_model_B_t_A_, &att_receiver_cov_scale_);

        Eigen::Vector3d prior_noise_rot_IB, prior_noise_I_t_B;
        prior_noise_rot_IB = getVectorFromParams("prior_noise_rot_IB");
        prior_noise_I_t_B = getVectorFromParams("prior_noise_I_t_B");
        prior_noise_model_T_I_B_ = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << prior_noise_rot_IB, prior_noise_I_t_B).finished());
        prior_noise_model_T_I_B_->print("prior_noise_model_T_I_B: ");

        Eigen::Vector3d prior_noise_I_v_B;
        prior_noise_I_v_B = getVectorFromParams("prior_noise_I_v_B");
        prior_noise_model_I_v_B_ = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << prior_noise_I_v_B).finished());
        prior_noise_model_I_v_B_->print("prior_noise_model_I_v_B: ");

        Eigen::Vector3d prior_noise_acc_bias, prior_noise_gyro_bias;
        prior_noise_acc_bias = getVectorFromParams("prior_noise_acc_bias");
        prior_noise_gyro_bias = getVectorFromParams("prior_noise_gyro_bias");
        prior_noise_model_imu_bias_ = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << prior_noise_acc_bias, prior_noise_gyro_bias)
                        .finished());
        prior_noise_model_imu_bias_->print("prior_noise_model_imu_bias: ");

        Eigen::Vector3d prior_acc_bias, prior_gyro_bias;
        prior_acc_bias = getVectorFromParams("prior_acc_bias");
        prior_gyro_bias = getVectorFromParams("prior_gyro_bias");
        prev_bias_ = gtsam::imuBias::ConstantBias(prior_acc_bias, prior_gyro_bias);
        prev_bias_.print("prior_imu_bias: ");

        double bias_acc_sigma = 0.0, bias_omega_sigma = 0.0, bias_acc_int_sigma = 0.0,
                bias_omega_int_sigma = 0.0, acc_sigma = 0.0, integration_sigma = 0.0,
                gyro_sigma = 0.0;
        bool use_2nd_order_coriolis = false;
        nh_private_.getParam("bias_acc_sigma", bias_acc_sigma);
        nh_private_.getParam("bias_omega_sigma", bias_omega_sigma);
        nh_private_.getParam("bias_acc_int_sigma", bias_acc_int_sigma);
        nh_private_.getParam("bias_omega_int_sigma", bias_omega_int_sigma);
        nh_private_.getParam("acc_sigma", acc_sigma);
        nh_private_.getParam("integration_sigma", integration_sigma);
        nh_private_.getParam("gyro_sigma", gyro_sigma);
        nh_private_.getParam("use_2nd_order_coriolis", use_2nd_order_coriolis);

        const gtsam::Matrix I = gtsam::Matrix33::Identity();
        auto imu_params =
                gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
        imu_params->biasAccCovariance = I * pow(bias_acc_sigma, 2);
        imu_params->biasOmegaCovariance = I * pow(bias_omega_sigma, 2);
        gtsam::Matrix bias_acc_omega_int = gtsam::Matrix66::Zero();
        bias_acc_omega_int.block<3, 3>(0, 0) = I * pow(bias_acc_int_sigma, 2);
        bias_acc_omega_int.block<3, 3>(3, 3) = I * pow(bias_omega_int_sigma, 2);
        imu_params->biasAccOmegaInt = bias_acc_omega_int;
        imu_params->accelerometerCovariance = I * pow(acc_sigma, 2);
        imu_params->integrationCovariance = I * pow(integration_sigma, 2);
        imu_params->gyroscopeCovariance = I * pow(gyro_sigma, 2);
        imu_params->use2ndOrderCoriolis = use_2nd_order_coriolis;
        imu_params->print("IMU settings: ");
        integrator_ =
                gtsam::PreintegratedCombinedMeasurements(imu_params, prev_bias_);

        // ISAM2
        gtsam::ISAM2Params parameters;
        // TODO(rikba): Set more ISAM2 params here.
        double relinearize_threshold_rot, relinearize_threshold_pos,
                relinearize_threshold_vel, relinearize_threshold_acc_bias,
                relinearize_threshold_gyro_bias,
                relinearize_threshold_antenna_calibration;
        nh_private_.getParam("isam2/relinearize_threshold_rot",
                             relinearize_threshold_rot);
        nh_private_.getParam("isam2/relinearize_threshold_pos",
                             relinearize_threshold_pos);
        nh_private_.getParam("isam2/relinearize_threshold_vel",
                             relinearize_threshold_vel);
        nh_private_.getParam("isam2/relinearize_threshold_acc_bias",
                             relinearize_threshold_acc_bias);
        nh_private_.getParam("isam2/relinearize_threshold_gyro_bias",
                             relinearize_threshold_gyro_bias);
        nh_private_.getParam("isam2/relinearize_threshold_antenna_calibration",
                             relinearize_threshold_antenna_calibration);
        gtsam::FastMap<char, gtsam::Vector> thresholds;
        thresholds['x'] =
                (gtsam::Vector(6) << Eigen::Vector3d::Constant(relinearize_threshold_rot),
                        Eigen::Vector3d::Constant(relinearize_threshold_pos))
                        .finished();
        thresholds['v'] = Eigen::Vector3d::Constant(relinearize_threshold_vel);
        thresholds['b'] = (gtsam::Vector(6) << Eigen::Vector3d::Constant(
                relinearize_threshold_acc_bias),
                Eigen::Vector3d::Constant(relinearize_threshold_gyro_bias))
                .finished();
        thresholds['p'] =
                Eigen::Vector3d::Constant(relinearize_threshold_antenna_calibration);
        thresholds['a'] =
                Eigen::Vector3d::Constant(relinearize_threshold_antenna_calibration);
        parameters.relinearizeThreshold = thresholds;
        nh_private_.getParam("isam2/relinearize_skip", parameters.relinearizeSkip);
        nh_private_.getParam("isam2/enable_partial_relinarization_check",
                             parameters.enablePartialRelinearizationCheck);

        double smoother_lag = 0.0;
        nh_private_.getParam("isam2/smoother_lag", smoother_lag);

        smoother_ = gtsam::IncrementalFixedLagSmoother(smoother_lag, parameters);

        // Subscribe to topics.
        const uint32_t kQueueSize = 1000;
        imu_sub_ =
                nh_.subscribe("imu0", kQueueSize, &MavStateEstimator::imuCallback, this);
        ROS_INFO("Subscribing to: %s", imu_sub_.getTopic().c_str());
        pos_0_sub_ =
                nh_.subscribe("pos0", kQueueSize, &MavStateEstimator::posCallback, this);
        ROS_INFO("Subscribing to: %s", pos_0_sub_.getTopic().c_str());
        baseline_sub_ = nh_.subscribe("baseline0", kQueueSize,
                                      &MavStateEstimator::baselineCallback, this);
        ROS_INFO("Subscribing to: %s", baseline_sub_.getTopic().c_str());

        // Advertise topics.
        timing_pub_ = nh_private_.advertise<mav_state_estimation::Timing>(
                "solveThreaded", kQueueSize);
        prediction_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
                "prediction", kQueueSize);
        optimization_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
                "optimization", kQueueSize);
        acc_bias_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
                "acc_bias", kQueueSize);
        gyro_bias_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
                "gyro_bias", kQueueSize);
        position_antenna_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
                "position_antenna", kQueueSize);
        attitude_antenna_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>(
                "attitude_antenna", kQueueSize);
        odometry_pub_ =
                nh_private_.advertise<nav_msgs::Odometry>("odometry", kQueueSize);
    }

    void MavStateEstimator::loadGnssParams(
            const std::string &antenna_ns, const gtsam::Symbol &symbol,
            gtsam::Point3 *B_t, gtsam::noiseModel::Isotropic::shared_ptr *process_noise,
            double *cov_scale) {
        assert(B_t);
        assert(cov_scale);

        // General GNSS parameters.
        *B_t = getVectorFromParams(antenna_ns + "/B_t");
        ROS_INFO_STREAM("Initial guess " << antenna_ns.c_str()
                                         << " location: " << B_t->transpose());
        int rate = 0;
        nh_private_.getParam(antenna_ns + "/rate", rate);
        ROS_INFO_STREAM(antenna_ns.c_str() << " rate: " << rate << " Hz");
        // addSensorTimes(rate);
        nh_private_.getParam(antenna_ns + "/scale_cov", *cov_scale);
        ROS_INFO_STREAM(antenna_ns.c_str() << " cov scale: " << *cov_scale);
    }

    Eigen::Vector3d MavStateEstimator::getVectorFromParams(
            const std::string &param) const {
        std::vector<double> vec;
        nh_private_.getParam(param, vec);
        Eigen::Vector3d eig = Eigen::Vector3d::Zero();
        if (vec.size() == 3) {
            eig = Eigen::Vector3d(vec.data());
        } else {
            ROS_WARN("Cannot process parameter: %s Vector size: %lu", param.c_str(),
                     vec.size());
        }
        return eig;
    }


    void MavStateEstimator::initializeState() {
        geometry_msgs::TransformStamped T_IB_0;
        if (init_.getInitialPose(&T_IB_0)) {
            // Get initial values.
            Eigen::Vector3d I_t_B;
            Eigen::Quaterniond q_IB;
            tf::vectorMsgToEigen(T_IB_0.transform.translation, I_t_B);
            tf::quaternionMsgToEigen(T_IB_0.transform.rotation, q_IB);
            Eigen::Vector3d I_v_B = Eigen::Vector3d::Zero();
            gtsam::Pose3 T_IB(gtsam::Rot3(q_IB), I_t_B);

            // Fill initial ISAM state.
            new_values_.insert(X(0), T_IB);
            new_values_.insert(V(0), I_v_B);
            new_values_.insert(B(0), prev_bias_);

            prev_state_ = gtsam::NavState(T_IB, I_v_B);

            auto prior_pose = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                    X(0), T_IB, prior_noise_model_T_I_B_);
            auto prior_vel = boost::make_shared<gtsam::PriorFactor<gtsam::Velocity3>>(
                    V(0), I_v_B, prior_noise_model_I_v_B_);
            auto prior_bias =
                    boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
                            B(0), prev_bias_, prior_noise_model_imu_bias_);
            new_graph_.add(prior_pose);
            new_graph_.add(prior_vel);
            new_graph_.add(prior_bias);

            // Initialize time stamps.
            stamp_to_idx_[T_IB_0.header.stamp] = 0;
            idx_to_stamp_[stamp_to_idx_[T_IB_0.header.stamp]] = T_IB_0.header.stamp;
            // Print
            new_values_.print("Initial state: ");
            ROS_INFO_STREAM("Initialization stamp: " << idx_to_stamp_[idx_]);
        }
    }

    void MavStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {

        // Rotate IMU from FRD to ENU ( I guess?)
        Eigen::Matrix3d R_enu_frd(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
        Eigen::Vector3d lin_acc_frd, ang_vel_frd, lin_acc_enu, ang_vel_enu;
        tf::vectorMsgToEigen(imu_msg->linear_acceleration, lin_acc_frd);
        tf::vectorMsgToEigen(imu_msg->angular_velocity, ang_vel_frd);
        lin_acc_enu = R_enu_frd * lin_acc_frd;
        ang_vel_enu = R_enu_frd * ang_vel_frd;


        // Do not update prediction while initial values are updated.
        std::unique_lock<std::recursive_mutex> lock(update_mtx_);
        ROS_INFO_ONCE("Received first IMU message.");
        if (!isInitialized()) {
            // Gravitational acceleration in inertial frame (ENU).
            Eigen::Vector3d I_g = integrator_.p().getGravity();
            // Gravitational acceleration in base frame (IMU).
            lin_acc_enu = prev_bias_.correctAccelerometer(lin_acc_enu);
            lin_acc_enu *= -1.0; // why this?
            base_frame_ = imu_msg->header.frame_id;
            ROS_INFO_ONCE("Base frame: %s", base_frame_.c_str());
            init_.setBaseFrame(base_frame_);
            init_.addOrientationConstraint1(I_g, lin_acc_enu, imu_msg->header.stamp);
            initializeState();
        } else if (imu_msg->header.stamp > prev_ts_) {
            // Integrate IMU (zero-order-hold).

            double dt = (imu_msg->header.stamp - prev_ts_).toSec();
            integrator_.integrateMeasurement(prev_lin_acc_, prev_ang_vel_, dt);

            if (imu_msg->header.seq % 10 == 0) {
                // Setup new inbetween IMU factor.
                addImuStamp(imu_msg->header.stamp);
                idx_ = stamp_to_idx_[imu_msg->header.stamp];
                auto imu_state = integrator_.predict(prev_state_, prev_bias_);


                new_values_.insert(B(idx_), prev_bias_);
                new_values_.insert(X(idx_), prev_state_.pose());
                new_values_.insert(V(idx_), prev_state_.v());

                auto imu_factor = boost::make_shared<gtsam::CombinedImuFactor>(
                        X(idx_ - 1), V(idx_ - 1), X(idx_), V(idx_), B(idx_ - 1), B(idx_),
                        integrator_);
                integrator_.resetIntegrationAndSetBias(prev_bias_);
                new_graph_.add(imu_factor);

                prev_state_ = imu_state;
                // Publish high rate IMU prediction.
                /* if ((imu_msg->header.seq % odometry_throttle_) == 0) {
                     geometry_msgs::TransformStamped T_IB =
                             getTransform(imu_state, imu_msg->header.stamp, base_frame_);
                     tfb_.sendTransform(T_IB);
                     prediction_pub_.publish(T_IB);
                     publishOdometry(imu_state.velocity(), ang_vel, prev_bias_,
                                     imu_msg->header.stamp, T_IB);
                 }*/
            }
            // Attempt to run solver thread.

            //solve();

        } else {
            ROS_ERROR("Cannot handle IMU message.");
            ROS_ERROR_STREAM("This IMU stamp: " << imu_msg->header.stamp);
        }
        prev_ts_ = imu_msg->header.stamp;
        prev_lin_acc_ = lin_acc_enu;
        prev_ang_vel_ = ang_vel_enu;
    }

    void MavStateEstimator::addImuStamp(const ros::Time &time) {
        idx_++;
        idx_to_stamp_[idx_] = time;
        stamp_to_idx_[time] = idx_;

        new_timestamps_[B(idx_)] = time.toSec();
        new_timestamps_[X(idx_)] = time.toSec();
        new_timestamps_[V(idx_)] = time.toSec();
    }

    void MavStateEstimator::posCallback(
            const piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr &pos_msg) {
        static piksi_rtk_msgs::PositionWithCovarianceStamped::ConstPtr prev_msg =
                nullptr;
        // Do not update factors while initial values are updated.
        std::unique_lock<std::recursive_mutex> lock(update_mtx_);
        ROS_INFO_ONCE("Received first POS message.");
        Eigen::Vector3d I_t_P;
        tf::pointMsgToEigen(pos_msg->position.position, I_t_P);
        if (!isInitialized()) {
            // GNSS antenna position in inertial frame (ENU).
            inertial_frame_ = pos_msg->header.frame_id;
            ROS_INFO_ONCE("Inertial frame: %s", inertial_frame_.c_str());
            init_.setInertialFrame(inertial_frame_);
            init_.addPositionConstraint(I_t_P, B_t_P_, pos_msg->header.stamp);
        } else {
            const bool kSmart = false;
            auto cov = gtsam::noiseModel::Gaussian::Covariance(
                    pos_receiver_cov_scale_ /
                    (pos_msg->header.stamp - prev_msg->header.stamp).toSec() *
                    Matrix3dRow::Map(pos_msg->position.covariance.data()),
                    kSmart);
            gtsam::NonlinearFactor::shared_ptr pos_factor;

            pos_factor = boost::make_shared<AbsolutePositionFactor1>(X(idx_), I_t_P,
                                                                     B_t_P_, cov);
            new_graph_.add(pos_factor);


            solve();
        }
        prev_msg = pos_msg;
    }

    void MavStateEstimator::baselineCallback(
            const libsbp_ros_msgs::MsgBaselineHeading::ConstPtr &baseline_msg) {
        static libsbp_ros_msgs::MsgBaselineHeading::ConstPtr prev_msg = nullptr;

        static ros::Time previous_stamp;
        // Do not update factors while initial values are updated.
        std::unique_lock<std::recursive_mutex> lock(update_mtx_);
        ROS_INFO_ONCE("Received first BASELINE message.");

        // message is in mili degrees
        double NED_heading = (baseline_msg->heading / 1000.0) * (M_PI / 180);
        ros::Time stamp =
                ros::Time::now();  // :-( :-( :-( but this stupid message has no
        // timestamp. should've used a different one...

        // TODO(rikba): Use ECEF frame by default.
        // TODO(rikba): Account for different frame positions. This rotation is
        // only correct if ENU and NED origin coincide.

        Eigen::Matrix3d r_I_B_yaw_measured(
        Eigen::AngleAxisd(NED_heading, Eigen::Vector3d::UnitZ()));
        Eigen::Vector3d hdg_measured = r_I_B_yaw_measured * Eigen::Vector3d::UnitY();

        // Moving baseline heading in base frame (IMU).
        const Eigen::Vector3d B_t_PA = B_t_A_ - B_t_P_;
        if (!isInitialized()) {
            init_.addOrientationConstraint2(hdg_measured, B_t_PA, stamp);
        } else {
            Eigen::Matrix3d covariance_simple;

            // doesn't make a whole lot of sense. to be checked later.
            covariance_simple.diagonal() << 0.04, 0.04, 0.000001;
            auto cov = gtsam::noiseModel::Gaussian::Covariance(
                    covariance_simple / (stamp - previous_stamp).toSec());
            gtsam::NonlinearFactor::shared_ptr baseline_factor;

            baseline_factor =
                    boost::make_shared<HeadingFactor1>(X(idx_), NED_heading, B_t_PA, cov);

            new_graph_.add(baseline_factor);
        }
        prev_msg = baseline_msg;
        previous_stamp = stamp;
    }

    void MavStateEstimator::publishOdometry(
            const Eigen::Vector3d &v_I, const Eigen::Vector3d &omega_B,
            const gtsam::imuBias::ConstantBias &bias, const ros::Time &stamp,
            const geometry_msgs::TransformStamped &T_IB) const {
        assert(T_IB.header.stamp == stamp);
        assert(T_IB.header.frame_id == inertial_frame_);
        assert(T_IB.child_frame_id == base_frame_);

        nav_msgs::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = inertial_frame_;
        msg.child_frame_id = "FLU";  // Publish odometry in FLU frame.

        try {
            geometry_msgs::TransformStamped T_BF;
            T_BF = tf_buffer_.lookupTransform(base_frame_, msg.child_frame_id, stamp);
            tf2::Stamped<tf2::Transform> tf2_T_IB, tf2_T_BF;
            tf2::fromMsg(T_IB, tf2_T_IB);
            tf2::fromMsg(T_BF, tf2_T_BF);

            // Pose
            auto tf2_T_IF = tf2_T_IB * tf2_T_BF;

            auto t_IF = tf2::toMsg(tf2_T_IF.getOrigin());
            Eigen::Vector3d p_IF;
            tf::vectorMsgToEigen(t_IF, p_IF);
            tf::pointEigenToMsg(p_IF, msg.pose.pose.position);

            msg.pose.pose.orientation = tf2::toMsg(tf2_T_IF.getRotation());

            // Twist
            tf::vectorEigenToMsg(v_I, msg.twist.twist.linear);
            tf2::Vector3 tf_v_I;
            tf2::fromMsg(msg.twist.twist.linear, tf_v_I);
            // TODO(rikba): Account for possible lever arm omega x r
            msg.twist.twist.linear =
                    tf2::toMsg(tf2::quatRotate(tf2_T_IF.getRotation().inverse(), tf_v_I));

            auto omega_B_corrected = omega_B - bias.gyroscope();
            tf::vectorEigenToMsg(omega_B_corrected, msg.twist.twist.angular);
            tf2::Vector3 tf_omega_B;
            tf2::fromMsg(msg.twist.twist.angular, tf_omega_B);
            msg.twist.twist.angular = tf2::toMsg(
                    tf2::quatRotate(tf2_T_BF.getRotation().inverse(), tf_omega_B));
        } catch (...) {
            ROS_WARN_THROTTLE(10.0,
                              "No odometry. Cannot lookup T_BF from B: %s to F: %s",
                              base_frame_.c_str(), msg.child_frame_id.c_str());
        }

        odometry_pub_.publish(msg);
    }

    geometry_msgs::TransformStamped MavStateEstimator::getTransform(
            const gtsam::NavState &state, const ros::Time &stamp,
            const std::string &child_frame_id) const {
        geometry_msgs::TransformStamped T_IB;
        T_IB.header.stamp = stamp;
        T_IB.header.frame_id = inertial_frame_;
        T_IB.child_frame_id = child_frame_id;

        tf::vectorEigenToMsg(state.position(), T_IB.transform.translation);
        tf::quaternionEigenToMsg(state.attitude().toQuaternion(),
                                 T_IB.transform.rotation);

        return T_IB;
    }

    void MavStateEstimator::publishAntennaPosition(
            const gtsam::Point3 &B_t, const ros::Time &stamp,
            const ros::Publisher &pub) const {
        geometry_msgs::Vector3Stamped antenna_position;
        antenna_position.header.stamp = stamp;
        antenna_position.header.frame_id = base_frame_;
        tf::vectorEigenToMsg(B_t, antenna_position.vector);
        pub.publish(antenna_position);
    }

    void MavStateEstimator::createBiasMessage(
            const gtsam::imuBias::ConstantBias &bias, const ros::Time &stamp,
            geometry_msgs::Vector3Stamped *acc_bias,
            geometry_msgs::Vector3Stamped *gyro_bias) const {
        assert(acc_bias);
        assert(gyro_bias);

        acc_bias->header.stamp = stamp;
        gyro_bias->header.stamp = stamp;

        tf::vectorEigenToMsg(bias.accelerometer(), acc_bias->vector);
        tf::vectorEigenToMsg(bias.gyroscope(), gyro_bias->vector);
    }

    void MavStateEstimator::publishBias(
            const gtsam::imuBias::ConstantBias &bias, const ros::Time &stamp,
            const ros::Publisher &acc_bias_pub, const ros::Publisher &gyro_bias_pub,
            geometry_msgs::Vector3Stamped *acc_bias,
            geometry_msgs::Vector3Stamped *gyro_bias) const {
        assert(acc_bias);
        assert(gyro_bias);
        createBiasMessage(bias, stamp, acc_bias, gyro_bias);

        acc_bias_pub.publish(*acc_bias);
        gyro_bias_pub.publish(*gyro_bias);
    }

    void MavStateEstimator::publishBias(const gtsam::imuBias::ConstantBias &bias,
                                        const ros::Time &stamp,
                                        const ros::Publisher &acc_bias_pub,
                                        const ros::Publisher &gyro_bias_pub) const {
        geometry_msgs::Vector3Stamped acc_bias, gyro_bias;
        publishBias(bias, stamp, acc_bias_pub, gyro_bias_pub, &acc_bias, &gyro_bias);
    }

    MavStateEstimator::~MavStateEstimator() {
        if (solver_thread_.joinable()) solver_thread_.join();
    }

    void MavStateEstimator::solve() {


        // Check for new result.
        if (is_solving_.load() || new_graph_.empty()) {
            return;
        }
        //  if (solver_thread_.joinable()) solver_thread_.join();

        // Copy new factors to graph.
        auto graph = std::make_unique<gtsam::NonlinearFactorGraph>(new_graph_);
        auto values = std::make_unique<gtsam::Values>(new_values_);
        auto stamps = std::make_unique<gtsam::FixedLagSmoother::KeyTimestampMap>(
                new_timestamps_);
        auto time = std::make_unique<ros::Time>(idx_to_stamp_[idx_]);
        new_graph_.resize(0);
        new_values_.clear();
        new_timestamps_.clear();

        // Solve.
        is_solving_.store(true);
        /*solver_thread_ =
                std::thread(&MavStateEstimator::solveThreaded, this, std::move(graph),
                            std::move(values), std::move(stamps), std::move(time), idx_);*/

        MavStateEstimator::solveThreaded(std::move(graph),
                                         std::move(values), std::move(stamps), std::move(time),
                                         idx_);


    }

    void MavStateEstimator::solveThreaded(
            std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
            std::unique_ptr<gtsam::Values> values,
            std::unique_ptr<gtsam::FixedLagSmoother::KeyTimestampMap> stamps,
            std::unique_ptr<ros::Time> time, const uint64_t i) {

        ROS_WARN_STREAM("Graph size " << graph->nrFactors());
        ROS_WARN_STREAM("Value size " << values->size());
        ROS_WARN_STREAM("Stamp size " << stamps->size());


        /*
        char name[] = "/tmp/graphXXXXXX";
        mkstemp(name);
        std::ofstream graph_file;
        graph_file.open(name);
        smoother_.getFactors().saveGraph(graph_file, *values);
        graph_file.close();
        ROS_WARN_STREAM("Logged to " << name);
        */

        assert(graph);
        assert(values);
        assert(stamps);
        assert(time);

        // Solve iterative problem.
        gttic_(solveThreaded);
        ROS_WARN("Start  update");


        auto ts = smoother_.timestamps();
        ROS_WARN_STREAM("size stamps " << ts.size());
        ROS_WARN_STREAM("size factors " << smoother_.getFactors().keys().size());
/*
        for (auto key_val_pair : ts){
            ROS_WARN_STREAM(key_val_pair.first << " " << (int)key_val_pair.second);
        }
        ROS_WARN_STREAM("-------------");
        for (auto key : smoother_.getFactors().keys()){
            ROS_WARN_STREAM(key);
        }*/

        smoother_.update(*graph, *values/*, *stamps*/);

        ROS_WARN("Done  update");
        auto pose = smoother_.calculateEstimate<gtsam::Pose3>(X(i));
        auto velocity = smoother_.calculateEstimate<gtsam::Velocity3>(V(i));
        auto bias = smoother_.calculateEstimate<gtsam::imuBias::ConstantBias>(B(i));

        ROS_WARN_STREAM("Pose " << pose.translation().transpose());
        ROS_WARN_STREAM("Bias");
        bias.print();


        gttoc_(solveThreaded);
        gtsam::tictoc_finishedIteration_();

        // Update new values (threadsafe, blocks all sensor callbacks).
        gtsam::NavState new_state(pose, velocity);
        std::unique_lock<std::recursive_mutex> lock(update_mtx_);
        {
            prev_state_ = new_state;
            prev_bias_ = bias;

            auto idx = i;
            for (auto it = new_graph_.begin(); it != new_graph_.end(); ++it) {
                auto imu = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(*it);
                if (imu) {
                    prev_state_ =
                            imu->preintegratedMeasurements().predict(prev_state_, prev_bias_);
                    new_values_.update(X(idx + 1), prev_state_.pose());
                    new_values_.update(V(idx + 1), prev_state_.velocity());
                    new_values_.update(B(idx + 1), prev_bias_);
                    idx++;
                }
            }
        }

        // ROS publishers
        tictoc_getNode(solveThreaded, solveThreaded);
        timing_msg_.header.stamp = *time;
        timing_msg_.iteration = solveThreaded->self() - timing_msg_.time;
        timing_msg_.time = solveThreaded->self();
        timing_msg_.min = solveThreaded->min();
        timing_msg_.max = solveThreaded->max();
        timing_msg_.mean = solveThreaded->mean();
        timing_pub_.publish(timing_msg_);

        geometry_msgs::TransformStamped T_IB =
                getTransform(new_state, *time, base_frame_ + "_optimization");
        tfb_.sendTransform(T_IB);
        optimization_pub_.publish(T_IB);
        publishBias(bias, *time, acc_bias_pub_, gyro_bias_pub_);

        is_solving_.store(false);
    }

}  // namespace mav_state_estimation
