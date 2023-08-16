// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <memory>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <utility>
#include <vector>

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2 headers
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <GeographicLib/LocalCartesian.hpp>
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

#include "SophusGtsamConverter.h"
#include <tf2_eigen/tf2_eigen.hpp>

using gtsam::symbol_shorthand::B;// Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::G;// GPS pose
using gtsam::symbol_shorthand::V;// Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;// Pose3 (x,y,z,r,p,y)

using namespace std::chrono_literals;
namespace kiss_icp_ros {

    using utils::EigenToPointCloud2;
    using utils::GetTimestamps;
    using utils::PointCloud2ToEigen;

    OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
        : rclcpp::Node("odometry_node", options), gps_buffer_(100) {
        child_frame_ = declare_parameter<std::string>("child_frame", child_frame_);
        odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
        publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
        config_.max_range = declare_parameter<double>("max_range", config_.max_range);
        config_.min_range = declare_parameter<double>("min_range", config_.min_range);
        config_.deskew = declare_parameter<bool>("deskew", config_.deskew);
        config_.voxel_size = declare_parameter<double>("voxel_size", config_.max_range / 100.0);
        config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
        config_.initial_threshold = declare_parameter<double>("initial_threshold", config_.initial_threshold);
        config_.min_motion_th = declare_parameter<double>("min_motion_th", config_.min_motion_th);
        if (config_.max_range < config_.min_range) {
            RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
            config_.min_range = 0.0;
        }

        gps_config_.lat = declare_parameter<double>("lat", 29.605333);
        gps_config_.lon = declare_parameter<double>("lon", 106.307656);
        gps_config_.alt = declare_parameter<double>("alt", 250);
        auto variance_x = declare_parameter<double>("variance_x", 2.0);
        auto variance_y = declare_parameter<double>("variance_y", 2.0);
        auto variance_z = declare_parameter<double>("variance_z", 1e8);

        gps_config_.noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << variance_x, variance_y, variance_z).finished());


        gps_trans_.Reset(gps_config_.lat, gps_config_.lon, gps_config_.alt);


        // Construct the main KISS-ICP odometry node
        adaptive_threshold_ = std::make_unique<kiss_icp::AdaptiveThreshold>(config_.initial_threshold, config_.min_motion_th, config_.max_range);

        // Initialize subscribers

        auto lidar_cb_group =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto gps_cb_group =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


        auto lidarOpt = rclcpp::SubscriptionOptions();
        lidarOpt.callback_group = lidar_cb_group;
        auto gpsOpt = rclcpp::SubscriptionOptions();
        gpsOpt.callback_group = gps_cb_group;


        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "pointcloud_topic", rclcpp::SensorDataQoS(),
                std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1),
                lidarOpt);
        RCLCPP_INFO(get_logger(), "Subscribed to pointcloud_topic %s", pointcloud_sub_->get_topic_name());


        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                "fix", rclcpp::SensorDataQoS(),
                std::bind(&OdometryServer::gpsHandler, this, std::placeholders::_1),
                gpsOpt);
        RCLCPP_INFO(get_logger(), "Subscribed to gps %s", gps_sub_->get_topic_name());


        // Initialize publishers
        rclcpp::QoS qos(rclcpp::KeepLast{queue_size_});

        // TransientLocal qos
        rclcpp::QoS map_qos(rclcpp::KeepLast{1});
        map_qos.transient_local();

        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", map_qos);
        local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", qos);
        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", qos);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // Initialize trajectory publisher
        path_msg_.header.frame_id = odom_frame_;
        traj_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", qos);

        gtsam_path_publisher_ = create_publisher<nav_msgs::msg::Path>("gtsam_trajectory", qos);


        map_pub_timer_ = create_wall_timer(1s, [this]() {
            //            RCLCPP_INFO_STREAM(get_logger(), "map_pub_timer_");
            decltype(key_frames_) key_frames_copy;
            decltype(isam_poses_) isam_poses_copy;
            {
                std::scoped_lock lock(key_frames__mutex_, isam_mutex_);
                key_frames_copy = key_frames_;
                isam_poses_copy = isam_poses_;
            }

            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker delete_all;
            delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_all);


            kiss_icp::VoxelHashMap map(config_.voxel_size, 500, config_.max_points_per_voxel);

            std_msgs::msg::Header header;
            header.frame_id = odom_frame_;
            header.stamp = this->now();


            //
            //
            size_t maker_id = 0;
            for (const auto &key_frame: key_frames_copy) {
                auto opt_pose = isam_poses_copy.at(key_frame.id);
                map.Update(key_frame.points, gtsamPose3toSouphusSE3(opt_pose));

                // draw gps constrain
                if (key_frame.gps) {
                    visualization_msgs::msg::Marker gps_marker;
                    gps_marker.header = header;
                    gps_marker.action = visualization_msgs::msg::Marker::ADD;
                    gps_marker.type = visualization_msgs::msg::Marker::SPHERE;
                    gps_marker.pose.position.x = key_frame.gps->x();
                    gps_marker.pose.position.y = key_frame.gps->y();
                    gps_marker.pose.position.z = key_frame.gps->z();
                    gps_marker.pose.orientation.w = 1.0;
                    gps_marker.scale.x = 0.5;
                    gps_marker.scale.y = 0.5;
                    gps_marker.scale.z = 0.5;
                    gps_marker.color.a = 1.0;
                    gps_marker.color.r = 1.0;
                    gps_marker.color.g = 0.0;
                    gps_marker.color.b = 0.0;
                    gps_marker.id = maker_id;
                    maker_id++;
                    marker_array.markers.push_back(gps_marker);
                }
            }
            marker_publisher_->publish(marker_array);


            //            // Publish the map
            std_msgs::msg::Header local_map_header;
            local_map_header = header;
            map_publisher_->publish(std::move(EigenToPointCloud2(map.Pointcloud(), local_map_header)));
        });


        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam.reset(new gtsam::ISAM2(parameters));

        //give the initial pose
        gtsam::NonlinearFactorGraph gtSAMgraph;
        gtsam::Values initialEstimate;
        const double approx_level_variance = std::pow(M_PI / 2.0, 2);
        const double unknown_direction_variance = std::pow(M_PI, 2);
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << approx_level_variance, approx_level_variance, unknown_direction_variance, 1e8, 1e8, 1e8).finished());// rad*rad, meter*meter
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3::Identity(), priorNoise));
        initialEstimate.insert(0, gtsam::Pose3::Identity());
        isam->update(gtSAMgraph, initialEstimate);


        RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
    }


    void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        RCLCPP_INFO_STREAM(get_logger(), "RegisterFrame");


        static boost::circular_buffer<CloudWithId> cloud_buffer_(10);
        static size_t lidar_frame_id = 0;
        int64_t lidar_time = rclcpp::Time(msg->header.stamp).nanoseconds();
        sensor_msgs::msg::PointCloud2::SharedPtr msg_at_base(new sensor_msgs::msg::PointCloud2);
        try {
            auto lidar2base = tf_buffer_->lookupTransform(msg->header.frame_id, child_frame_, tf2::TimePointZero);// if the tf is not static, we should look at the frame time
            tf2::doTransform(*msg, *msg_at_base, lidar2base);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s 2 %s tf is not available %s", msg->header.frame_id.c_str(), child_frame_.c_str(), ex.what());
            return;
        }

        lidar_frame_id++;


        const auto points = PointCloud2ToEigen(msg_at_base);

        // todo: add deskew stuff later
        //        const auto timestamps = [&]() -> std::vector<double> {
        //            if (!config_.deskew) return {};
        //            return GetTimestamps(msg);
        //        }();

        // Preprocess the input cloud
        const auto &cropped_frame = kiss_icp::Preprocess(points, config_.max_range, config_.min_range);
        // Voxelize
        const auto frame_downsample = kiss_icp::VoxelDownsample(cropped_frame, config_.voxel_size * 0.5);
        const auto source = kiss_icp::VoxelDownsample(frame_downsample, config_.voxel_size * 1.5);

        // Get motion prediction and adaptive_threshold
        const double sigma = GetAdaptiveThreshold(lidar_frame_id);

        // Compute initial_guess for ICP
        isam_mutex_.lock();
        const auto prediction = GetPredictionModel(lidar_frame_id);
        const auto last_pose = isam_poses_.contains(lidar_frame_id - 1) ? gtsamPose3toSouphusSE3(isam_poses_.at(lidar_frame_id - 1)) : Sophus::SE3d();
        const auto initial_guess = last_pose * prediction;
        kiss_icp::VoxelHashMap local_map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel);
        for (const auto &cloud_with_id: cloud_buffer_) {
            local_map.Update(cloud_with_id.points, gtsamPose3toSouphusSE3(isam_poses_.at(cloud_with_id.id)));
        }
        isam_mutex_.unlock();


        // Run icp
        const Sophus::SE3d pose = kiss_icp::RegisterFrame(source,       //
                                                          local_map,    //
                                                          initial_guess,//
                                                          3.0 * sigma,  //
                                                          sigma / 3.0);
        const auto model_deviation = initial_guess.inverse() * pose;
        adaptive_threshold_->UpdateModelDeviation(model_deviation);
        cloud_buffer_.push_back({lidar_frame_id, frame_downsample});


        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
        gtsam::Pose3 poseFrom = sophusSE3TogtsamPose3(last_pose);
        gtsam::Pose3 poseTo = sophusSE3TogtsamPose3(pose);
        gtsam::Pose3 delta = poseFrom.between(poseTo);


        gtsam::NonlinearFactorGraph gtSAMgraph;
        gtsam::Values initialEstimate;


        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(lidar_frame_id - 1, lidar_frame_id, delta, odometryNoise));
        initialEstimate.insert(lidar_frame_id, poseTo);


        std::optional<Eigen::Vector3d> current_gps;
        {
            std::lock_guard lock(gps_buffer_mutex_);
            auto gps_here_opt = gps_buffer_.get_msg_cloest(lidar_time, 100ms);
            if (gps_here_opt.has_value()) {
                const auto current_gps_ptr = gps_here_opt.value();
                current_gps = *current_gps_ptr;
            }
        }

        RCLCPP_INFO_STREAM(get_logger(), "find  gps at  " << lidar_time << " : " << current_gps.has_value());


        auto add_current_frame_as_key = [&lidar_time, &frame_downsample, &gtSAMgraph, &current_gps, this]() {
            if (current_gps) {

                gtsam::GPSFactor gps_factor(lidar_frame_id,// the latest key
                                            gtsam::Point3(current_gps->x(), current_gps->y(), current_gps->z()),
                                            gps_config_.noise);
                gtSAMgraph.add(gps_factor);
                RCLCPP_INFO_STREAM(get_logger(), "add gps constrain for " << lidar_frame_id << "  " << current_gps->transpose());
            }
            KeyFrameInfo key_frame;
            key_frame.gps = current_gps;
            key_frame.id = lidar_frame_id;
            key_frame.points = frame_downsample;
            key_frame.timestamp = lidar_time;
            key_frames_.push_back(key_frame);
        };


        {
            std::scoped_lock lock(isam_mutex_, key_frames__mutex_);
            if (key_frames_.empty()) {
                add_current_frame_as_key();
            } else {
                // check if add key frame is needed
                const auto &last_key_frame = key_frames_.back();
                const uint64_t delta_lidar_time_ns = (lidar_time - last_key_frame.timestamp);
                const auto last_key_frame_opt_pose = isam_poses_.at(last_key_frame.id);
                gtsam::Pose3 delta_k2c = last_key_frame_opt_pose.between(poseTo);

                if (delta_lidar_time_ns > 1e9 * 10) {// 10s
                    add_current_frame_as_key();
                } else if (delta_k2c.translation().norm() > 5) {// lidar odom move for  5m
                    add_current_frame_as_key();
                } else {
                    if (last_key_frame.gps.has_value() and current_gps.has_value()) {
                        double norm = (last_key_frame.gps.value() - current_gps.value()).norm();
                        if (norm > 5.0) {// gps move for 5m
                            add_current_frame_as_key();
                        }
                    }
                }
            }
        }


        isam->update(gtSAMgraph, initialEstimate);
        gtsam::Values estimate = isam->calculateEstimate();


        // store the latest optimized pose
        {
            std::lock_guard lock(isam_mutex_);
            for (const auto &key: estimate.keys()) {
                isam_poses_[key] = estimate.at<gtsam::Pose3>(key);
            }
        }


        std_msgs::msg::Header odom_header;
        odom_header.stamp = msg->header.stamp;
        odom_header.frame_id = odom_frame_;


        // publish trajectory msg
        nav_msgs::msg::Path path = gtsamValues2Path(estimate, odom_header);
        gtsam_path_publisher_->publish(path);


        // Convert from Eigen to ROS types
        const Eigen::Vector3d t_current = pose.translation();
        const Eigen::Quaterniond q_current = pose.unit_quaternion();

        // Broadcast the tf
        if (publish_odom_tf_) {
            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header = odom_header;
            transform_msg.child_frame_id = child_frame_;
            transform_msg.transform.rotation.x = q_current.x();
            transform_msg.transform.rotation.y = q_current.y();
            transform_msg.transform.rotation.z = q_current.z();
            transform_msg.transform.rotation.w = q_current.w();
            transform_msg.transform.translation.x = t_current.x();
            transform_msg.transform.translation.y = t_current.y();
            transform_msg.transform.translation.z = t_current.z();
            tf_broadcaster_->sendTransform(transform_msg);
            //            RCLCPP_INFO(get_logger(), "Broadcasted tf from %s to %s", odom_frame_.c_str(),
            //                        child_frame_.c_str());
        }


        auto local_map_header = msg->header;
        local_map_header.frame_id = odom_frame_;
        local_map_publisher_->publish(std::move(EigenToPointCloud2(local_map.Pointcloud(), local_map_header)));
    }

    void OdometryServer::gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gpsMsg) {
        int64_t time = rclcpp::Time(gpsMsg->header.stamp).nanoseconds();

        RCLCPP_INFO(get_logger(), "gpsHandler %ld", time);
        Eigen::Isometry3d gps2base;
        try {
            auto imu2base = tf_buffer_->lookupTransform(gpsMsg->header.frame_id, child_frame_, tf2::TimePointZero);// if the tf is not static, we should look at the frame time
            gps2base = tf2::transformToEigen(imu2base);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s 2 %s tf is not available %s", gpsMsg->header.frame_id.c_str(), child_frame_.c_str(), ex.what());
            return;
        }

        Eigen::Vector3d trans_local_;
        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        //        RCLCPP_INFO_STREAM(get_logger(), "gpsHandler " << trans_local_);

        if (gpsMsg->status.status != 0) {
            return;
        }

        trans_local_ = gps2base * trans_local_;

        {
            std::lock_guard<std::mutex> lock(gps_buffer_mutex_);
            gps_buffer_.put_in(time, trans_local_);
        }
    }
}// namespace kiss_icp_ros
