//
// Created by jimmy on 23-8-17.
//

#ifndef LIORF_SCLOOP_H
#define LIORF_SCLOOP_H

#include "marker_color.h"
#include <Scancontext.h>
#include <boost/thread/synchronized_value.hpp>
#include <gtsam/geometry/Pose3.h>
#include <pcl/registration/icp.h>
#include <visualization_msgs/msg/marker.hpp>
namespace kiss_icp_ros {
    class ScLoop {
        using PointType = SCPointType;

        struct LoopClosureData {
            size_t from_id;
            size_t to_id;
            Eigen::Isometry3d transform;
            double fitness;
            bool used{false};
        };

    public:
        void CalculateLoopClosure(size_t laser_id, std::vector<Eigen::Vector3d> points);

        void update_key_pose(const std::map<size_t, gtsam::Pose3> &new_poses) {
            auto lock = key_to_pose_dict.synchronize();


            for (const auto &[key, pose]: new_poses) {
                lock->operator[](key) = Eigen::Isometry3d(pose.matrix());
            }
        }
        std::vector<LoopClosureData> get_all_closure() {
            return loop_closure_data_.get();
        }

        std::vector<LoopClosureData> get_all_unused_closure();


        std::vector<visualization_msgs::msg::Marker> get_loop_closure_markers(std_msgs::msg::Header header, std::string ns = "loop_closure");


    private:
        SCManager sc_manager_;

        boost::synchronized_value<std::map<size_t, Eigen::Isometry3d>> key_to_pose_dict;

        std::mutex mutex_;
        std::map<size_t, int> key_to_sc_key_dict;
        std::map<int, size_t> sc_to_lidar_key_dict;

        std::map<int, pcl::PointCloud<SCPointType>::ConstPtr> sc_key_to_cloud_dict;

        boost::synchronized_value<std::vector<LoopClosureData>> loop_closure_data_;
    };
}// namespace kiss_icp_ros

#endif//LIORF_SCLOOP_H
