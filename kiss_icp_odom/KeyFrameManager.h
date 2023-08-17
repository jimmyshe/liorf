//
// Created by jimmy on 23-8-17.
//

#ifndef LIORF_KEYFRAMEMANAGER_H
#define LIORF_KEYFRAMEMANAGER_H

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <map>
#include <optional>
#include <ranges>
#include <visualization_msgs/msg/marker.hpp>
#include <Scancontext.h>

namespace kiss_icp_ros {

    struct KeyFrameInfo {
        size_t id;
        int64_t timestamp;
        std::vector<Eigen::Vector3d> points;
        std::optional<Eigen::Vector3d> gps;
    };


    struct PointsWithOrigin {
        gtsam::Pose3 pose;
        std::vector<Eigen::Vector3d> points;
    };

    class KeyFrameManager {


    public:
        void put_in(const KeyFrameInfo &key_frame_info, const gtsam::Pose3 &pose);


        void update_key_frame_pose(const std::map<size_t, gtsam::Pose3> &new_poses);

        double get_distance_to_last_gps(const Eigen::Vector3d &new_gps) const;

        double get_distance_to_last_key(const gtsam::Point3 &p) const;

        double get_angle_to_last_key(const gtsam::Rot3 &r) const;

        std::vector<visualization_msgs::msg::Marker> get_gps_markers(std_msgs::msg::Header header, std::string ns = "gps") const;

        std::vector<visualization_msgs::msg::Marker> get_key_markers(std_msgs::msg::Header header, std::string ns = "key_frame") const;


        std::map<size_t,PointsWithOrigin> get_map() const;

    private:
        mutable std::mutex mutex_;
        std::vector<KeyFrameInfo> key_frames_;
        std::map<size_t, gtsam::Pose3> key_frame_pose;
    };

}// namespace kiss_icp_ros
#endif//LIORF_KEYFRAMEMANAGER_H
