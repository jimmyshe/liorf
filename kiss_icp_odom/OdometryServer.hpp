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
#pragma once

// KISS-ICP
#include "kiss_icp/core/Deskew.hpp"
#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include <gtsam/nonlinear/ISAM2.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
// ROS 2
#include "MsgBuffer.h"
#include "SophusGtsamConverter.h"
#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <boost/circular_buffer.hpp>
#include <gtsam/geometry/Pose3.h>
namespace kiss_icp_ros {


    struct CloudWithId {
        size_t id;
        std::vector<Eigen::Vector3d> points;
    };

    struct GpsConfig{
        double lat;
        double lon;
        double alt;
        gtsam::noiseModel::Diagonal::shared_ptr noise;
    };


    class OdometryServer : public rclcpp::Node {
    public:
        /// OdometryServer constructor
        OdometryServer() = delete;
        explicit OdometryServer(const rclcpp::NodeOptions &options);

    private:
        /// Register new frame
        void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    private:
        void gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gpsMsg);


        double GetAdaptiveThreshold(size_t id) {
            if (!HasMoved(id)) {
                return config_.initial_threshold;
            }
            return adaptive_threshold_->ComputeThreshold();
        }

        Sophus::SE3d GetPredictionModel(size_t id) const {
            if (isam_poses_.contains(id - 1) and isam_poses_.contains(id - 2)) {
                return gtsamPose3toSouphusSE3(isam_poses_.at(id - 2).inverse().compose(isam_poses_.at(id - 1)));
            }
            Sophus::SE3d pred = Sophus::SE3d();
            return pred;
        }

        bool HasMoved(size_t id) {
            if (isam_poses_.contains(id - 1) and isam_poses_.contains(id - 2)) {
                auto motion = isam_poses_.at(id - 2).inverse().compose(isam_poses_.at(id - 1));
                return motion.translation().norm() > 5.0 * config_.min_motion_th;
            }
            return false;
        }

        //        Sophus::SE3d GetPredictionModel() const {
        //            Sophus::SE3d pred = Sophus::SE3d();
        //            const size_t N = poses_.size();
        //            if (N < 2) return pred;
        //            return poses_[N - 2].inverse() * poses_[N - 1];
        //        }
        //
        //        bool HasMoved() {
        //            if (poses_.empty()) return false;
        //            const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
        //            return motion > 5.0 * config_.min_motion_th;
        //        }

        //isam
        std::unique_ptr<gtsam::ISAM2> isam;

        std::mutex isam_mutex_;
        std::map<size_t, gtsam::Pose3> isam_poses_;

        //gps q
        std::mutex gps_buffer_mutex_;
        MsgBuffer<Eigen::Vector3d> gps_buffer_;


        //recent cloud buffer
        std::mutex cloud_buffer_mutex_;
        boost::circular_buffer<CloudWithId> cloud_buffer_;

        std::mutex cloud_map_mutex_;
        std::map<size_t, std::vector<Eigen::Vector3d>> cloud_map_;


        /// Ros node stuff
        size_t queue_size_{1};

        /// Tools for broadcasting TFs.
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        bool publish_odom_tf_{true};

        /// Data subscribers.
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;


        /// Data publishers.
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
//        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
//        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kpoints_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;


        /*!
         * @brief timer
         */
        rclcpp::TimerBase::SharedPtr map_pub_timer_;

        /// Path publisher
        nav_msgs::msg::Path path_msg_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gtsam_path_publisher_;


        /// KISS-ICP
        kiss_icp::pipeline::KISSConfig config_;
        GpsConfig gps_config_;
        // KISS-ICP pipeline modules
        std::unique_ptr<kiss_icp::AdaptiveThreshold> adaptive_threshold_;
        /// Global/map coordinate frame.
        std::string odom_frame_{"odom"};
        std::string child_frame_{"base_link"};


        GeographicLib::LocalCartesian gps_trans_;
    };

}// namespace kiss_icp_ros
