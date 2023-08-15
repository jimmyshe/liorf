#include "utility.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using gtsam::symbol_shorthand::B;// Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;// Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;// Pose3 (x,y,z,r,p,y)


class IMUPreintegration : public ParamServer {
public:
    std::mutex mtx;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subIncOdometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuIncOdometry;// incremental odometry

    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;// optimized (only used in odom call back)
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;// incremental

    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;


    // last state of imu optimized  integration (graph optimized with previous slam results)
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_ = gtsam::imuBias::ConstantBias();

    gtsam::NavState prevStateOdom;            // copy of prevState_ for odom integration
    gtsam::imuBias::ConstantBias prevBiasOdom;// copy of prevBias_ for odom integration

    bool doneFirstOpt = false;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;//tmp value
    gtsam::Values graphValues;               //tmp value

    const double delta_t = 0;

    int key = 1;


    IMUPreintegration(const rclcpp::NodeOptions &options) : ParamServer("lio_sam_imu_preintegration", options) {


        tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        callbackGroupImu = create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;


        subImu = create_subscription<sensor_msgs::msg::Imu>(
                imuTopic, qos_imu,
                std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1),
                imuOpt);
        subIncOdometry = create_subscription<nav_msgs::msg::Odometry>(
                "liorf/mapping/odometry_incremental", qos,
                std::bind(&IMUPreintegration::odometry_incremental_handler, this, std::placeholders::_1),
                odomOpt);
        pubImuIncOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic + "_incremental", qos_imu);


        boost::shared_ptr<gtsam::PreintegrationParams> imu_params = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        imu_params->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2);// acc white noise in continuous
        imu_params->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);    // gyro white noise in continuous
        imu_params->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-2, 2);         // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
        ;// assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());// rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                              // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                            // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(imu_params, prior_imu_bias);// setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(imu_params, prior_imu_bias);// setting up the IMU integration for optimization
    }

    void resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams() {
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void odometry_incremental_handler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        const double currentCorrectionTime = stamp2Sec(odomMsg->header.stamp);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty()) {
            return;
        }

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int) odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (!systemInitialized) {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty()) {
                if (stamp2Sec(imuQueOpt.front().header.stamp) < currentCorrectionTime - delta_t) {
                    lastImuT_opt = stamp2Sec(imuQueOpt.front().header.stamp);
                    imuQueOpt.pop_front();
                } else {
                    break;
                }
            }

            gtsam::NonlinearFactorGraph tmp_graphFactors;//tmp value
            gtsam::Values tmp_graphValues;               //tmp value


            // initial pose
            prevPose_ = base_pose;
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            tmp_graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            tmp_graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            tmp_graphFactors.add(priorBias);
            // add values
            tmp_graphValues.insert(X(0), prevPose_);
            tmp_graphValues.insert(V(0), prevVel_);
            tmp_graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(tmp_graphFactors, tmp_graphValues);


            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;
            return;
        }


        // reset graph for speed  // only keep 100 node in graph
        if (key == 100) {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
            // reset graph
            resetOptimization();

            gtsam::NonlinearFactorGraph tmp_graphFactors;//tmp value
            gtsam::Values tmp_graphValues;               //tmp value

            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            tmp_graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            tmp_graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            tmp_graphFactors.add(priorBias);
            // add values
            tmp_graphValues.insert(X(0), prevPose_);
            tmp_graphValues.insert(V(0), prevVel_);
            tmp_graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(tmp_graphFactors, tmp_graphValues);
            key = 1;
        }


        // 1. integrate all imu data before currentCorrectionTime and optimize
        while (!imuQueOpt.empty()) {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = stamp2Sec(thisImu->header.stamp);
            if (imuTime < currentCorrectionTime - delta_t) {
                double dt = (lastImuT_opt < 0) ? (1.0 / imuRate) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            } else {
                break;
            }
        }
        gtsam::NonlinearFactorGraph tmp_graphFactors;//tmp value
        gtsam::Values tmp_graphValues;               //tmp value
        // add imu factor to graph
        const auto &preIntegrated_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preIntegrated_imu);
        tmp_graphFactors.add(imu_factor);
        // add imu bias between factor
        tmp_graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key),
                gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = base_pose;
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        tmp_graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState prediction_by_imu = imuIntegratorOpt_->predict(prevState_, prevBias_);
        tmp_graphValues.insert(X(key), prediction_by_imu.pose());
        tmp_graphValues.insert(V(key), prediction_by_imu.v());
        tmp_graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(tmp_graphFactors, tmp_graphValues);
        optimizer.update();

        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_)) {
            resetParams();
            return;
        }


        // 2. after optimization, re-propagate imu odometry reintegration
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;// last imu time (for dt calculation)
        while (!imuQueImu.empty() && stamp2Sec(imuQueImu.front().header.stamp) < currentCorrectionTime - delta_t) {
            lastImuQT = stamp2Sec(imuQueImu.front().header.stamp);
            imuQueImu.pop_front();
        }
        // re propagate
        if (!imuQueImu.empty()) {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int) imuQueImu.size(); ++i) {
                sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = stamp2Sec(thisImu->header.stamp);
                double dt = (lastImuQT < 0) ? (1.0 / imuRate) : (imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 5) {
            RCLCPP_WARN(get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
            RCLCPP_WARN(get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {
        std::lock_guard<std::mutex> lock(mtx);

        geometry_msgs::msg::TransformStamped imu2base;
        try {
            imu2base = tf_buffer_->lookupTransform(baselinkFrame, imu_raw->header.frame_id, tf2::TimePointZero);
            Eigen::Isometry3d t = tf2::transformToEigen(imu2base);
            if (t.matrix() != Eigen::Isometry3d::Identity().matrix()) {
                RCLCPP_FATAL(this->get_logger(), "imu to base tf is not identity, please check your tf tree or imu mount");
                rclcpp::shutdown();
            }


        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Can not get imu tf to base %s", ex.what());
            return;
        }


        imuQueOpt.push_back(*imu_raw);
        imuQueImu.push_back(*imu_raw);


        if (!doneFirstOpt) {
            return;
        }

        double imuTime = stamp2Sec(imu_raw->header.stamp);
        static double last_imuTime = -1;
        double dt = (last_imuTime < 0) ? (1.0 / imuRate) : (imuTime - last_imuTime);
        last_imuTime = imuTime;


        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(imu_raw->linear_acceleration.x, imu_raw->linear_acceleration.y, imu_raw->linear_acceleration.z),
                                                gtsam::Vector3(imu_raw->angular_velocity.x, imu_raw->angular_velocity.y, imu_raw->angular_velocity.z), dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);


        // publish odometry
        auto odometry = nav_msgs::msg::Odometry();
        odometry.header.stamp = imu_raw->header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = baselinkFrame;

        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());

        odometry.pose.pose.position.x = imuPose.translation().x();
        odometry.pose.pose.position.y = imuPose.translation().y();
        odometry.pose.pose.position.z = imuPose.translation().z();
        odometry.pose.pose.orientation.x = imuPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = imuPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = imuPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = imuPose.rotation().toQuaternion().w();
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = imu_raw->angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = imu_raw->angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = imu_raw->angular_velocity.z + prevBiasOdom.gyroscope().z();


        auto cov = imuIntegratorImu_->preintMeasCov();

        odometry.pose.covariance[0] = cov(3, 3);
        odometry.pose.covariance[1 * 6 + 1] = cov(4, 4);
        odometry.pose.covariance[2 * 6 + 2] = cov(5, 5);
        odometry.pose.covariance[3 * 6 + 3] = cov(0, 0);
        odometry.pose.covariance[4 * 6 + 4] = cov(1, 1);
        odometry.pose.covariance[5 * 6 + 5] = cov(2, 2);


        pubImuIncOdometry->publish(odometry);
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor e;

    auto ImuP = std::make_shared<IMUPreintegration>(options);
    e.add_node(ImuP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> IMU Preintegration Started.\033[0m");

    e.spin();

    rclcpp::shutdown();
    return 0;
}
