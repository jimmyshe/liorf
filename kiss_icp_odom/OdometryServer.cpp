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
        : rclcpp::Node("odometry_node", options), gps_buffer_(100), cloud_buffer_(100) {
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

        // Construct the main KISS-ICP odometry node
        adaptive_threshold_ = std::make_unique<kiss_icp::AdaptiveThreshold>(config_.initial_threshold, config_.min_motion_th, config_.max_range);

        // Initialize subscribers
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "pointcloud_topic", rclcpp::SensorDataQoS(),
                std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Subscribed to pointcloud_topic %s", pointcloud_sub_->get_topic_name());


        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                "fix", rclcpp::SensorDataQoS(),
                std::bind(&OdometryServer::gpsHandler, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Subscribed to gps %s", gps_sub_->get_topic_name());


        // Initialize publishers
        rclcpp::QoS qos(rclcpp::KeepLast{queue_size_});

        // TransientLocal qos
        rclcpp::QoS map_qos(rclcpp::KeepLast{1});
        map_qos.transient_local();

        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", map_qos);
        local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", qos);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // Initialize trajectory publisher
        path_msg_.header.frame_id = odom_frame_;
        traj_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", qos);

        gtsam_path_publisher_ = create_publisher<nav_msgs::msg::Path>("gtsam_trajectory", qos);


        map_pub_timer_ = create_wall_timer(1s, [this]() {
            kiss_icp::VoxelHashMap map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel);


            {
                std::scoped_lock lock(cloud_map_mutex_, isam_mutex_);
                for (const auto &[id, cloud]: cloud_map_) {
                    map.Update(cloud, gtsamPose3toSouphusSE3(isam_poses.at(id)));
                }
            }

            // Publish the map
            std_msgs::msg::Header local_map_header;
            local_map_header.frame_id = odom_frame_;
            local_map_header.stamp = this->now();
            map_publisher_->publish(std::move(EigenToPointCloud2(map.Pointcloud(), local_map_header)));
        });


        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam.reset(new gtsam::ISAM2(parameters));

        //give the initial pose
        gtsam::NonlinearFactorGraph gtSAMgraph;
        gtsam::Values initialEstimate;
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << M_2_PI * M_2_PI, M_2_PI * M_2_PI, M_2_PI * M_2_PI, 1e8, 1e8, 1e8).finished());// rad*rad, meter*meter
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3::Identity(), priorNoise));
        initialEstimate.insert(0, gtsam::Pose3::Identity());
        isam->update(gtSAMgraph, initialEstimate);


        RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
    }


    void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        static size_t lidar_frame_id = 0;
        int64_t time = rclcpp::Time(msg->header.stamp).nanoseconds();
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
        const auto last_pose = isam_poses.contains(lidar_frame_id - 1) ? gtsamPose3toSouphusSE3(isam_poses.at(lidar_frame_id - 1)) : Sophus::SE3d();
        const auto initial_guess = last_pose * prediction;
        kiss_icp::VoxelHashMap local_map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel);
        for (const auto &cloud_with_id: cloud_buffer_) {
            local_map.Update(cloud_with_id.points, gtsamPose3toSouphusSE3(isam_poses.at(cloud_with_id.id)));
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
        {
            std::lock_guard lock(cloud_map_mutex_);
            cloud_map_[lidar_frame_id] = frame_downsample;
        }


        static std::optional<gtsam::Pose3> prev_lidar_optimized_pose{};

        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
        gtsam::Pose3 poseFrom = sophusSE3TogtsamPose3(last_pose);
        gtsam::Pose3 poseTo = sophusSE3TogtsamPose3(pose);
        gtsam::Pose3 delta = poseFrom.between(poseTo);


        gtsam::NonlinearFactorGraph gtSAMgraph;
        gtsam::Values initialEstimate;


        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(lidar_frame_id - 1, lidar_frame_id, delta, odometryNoise));
        initialEstimate.insert(lidar_frame_id, poseTo);


        auto add_gps_factor = [this, &gtSAMgraph](Eigen::Vector3d gps) {
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 2.0, 2.0, 2.0).finished());
            gtsam::GPSFactor gps_factor(lidar_frame_id,// the latest key
                                        gtsam::Point3(gps.x(), gps.y(), gps.z()),
                                        gps_noise);
            gtSAMgraph.add(gps_factor);
            RCLCPP_INFO_STREAM(get_logger(), "add gps constrain for " << lidar_frame_id << "  " << gps.transpose());
        };


        static std::optional<Eigen::Vector3d> last_gps{};
        {
            std::lock_guard lock(gps_buffer_mutex_);
            auto gps_here_opt = gps_buffer_.get_msg_cloest(time, 10ms);
            if (gps_here_opt.has_value()) {
                const auto &current_gps = *gps_here_opt.value();
                if (last_gps == std::nullopt) {
                    last_gps = current_gps;
                    add_gps_factor(current_gps);
                } else {
                    if ((last_gps.value() - current_gps).norm() > 5.0) {
                        last_gps = current_gps;
                        add_gps_factor(current_gps);
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
                isam_poses[key] = estimate.at<gtsam::Pose3>(key);
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

        // publish trajectory msg
        //        geometry_msgs::msg::PoseStamped pose_msg;
        //        pose_msg.pose.orientation.x = q_current.x();
        //        pose_msg.pose.orientation.y = q_current.y();
        //        pose_msg.pose.orientation.z = q_current.z();
        //        pose_msg.pose.orientation.w = q_current.w();
        //        pose_msg.pose.position.x = t_current.x();
        //        pose_msg.pose.position.y = t_current.y();
        //        pose_msg.pose.position.z = t_current.z();
        //        pose_msg.header.stamp = msg->header.stamp;
        //        pose_msg.header.frame_id = odom_frame_;
        //        path_msg_.poses.push_back(pose_msg);
        //        traj_publisher_->publish(path_msg_);
        //
        //        // publish odometry msg
        //        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        //        odom_msg->header = pose_msg.header;
        //        odom_msg->child_frame_id = child_frame_;
        //        odom_msg->pose.pose = pose_msg.pose;
        //        odom_publisher_->publish(std::move(odom_msg));
        //
        //        // Publish KISS-ICP internal data, just for debugging
        //        auto frame_header = msg->header;
        //        frame_header.frame_id = child_frame_;
        //        frame_publisher_->publish(std::move(EigenToPointCloud2(frame, frame_header)));
        //        kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, frame_header)));
        //
        // Map is referenced to the odometry_frame
        auto local_map_header = msg->header;
        local_map_header.frame_id = odom_frame_;
        local_map_publisher_->publish(std::move(EigenToPointCloud2(local_map.Pointcloud(), local_map_header)));
    }

    void OdometryServer::gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gpsMsg) {

        static GeographicLib::LocalCartesian gps_trans_;
        static bool first_gps = false;
        if (!first_gps) {
            first_gps = true;
            gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);//todo: set as a parameter
        }
        Eigen::Vector3d trans_local_;
        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        //        RCLCPP_INFO_STREAM(get_logger(), "gpsHandler " << trans_local_);

        if (gpsMsg->status.status != 0) {
            return;
        }

        int64_t time = rclcpp::Time(gpsMsg->header.stamp).nanoseconds();
        {
            std::lock_guard<std::mutex> lock(gps_buffer_mutex_);

            gps_buffer_.put_in(time, trans_local_);
        }
    }
}// namespace kiss_icp_ros
