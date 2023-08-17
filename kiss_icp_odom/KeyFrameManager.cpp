//
// Created by jimmy on 23-8-17.
//

#include "KeyFrameManager.h"
void kiss_icp_ros::KeyFrameManager::put_in(const kiss_icp_ros::KeyFrameInfo &key_frame_info, const gtsam::Pose3 &pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    key_frames_.push_back(key_frame_info);
    assert(key_frame_pose.contains(key_frame_info.id) == false);
    key_frame_pose[key_frame_info.id] = pose;
}
double kiss_icp_ros::KeyFrameManager::get_distance_to_last_gps(const Eigen::Vector3d &new_gps) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &key_frame: std::ranges::reverse_view(key_frames_)) {
        if (key_frame.gps.has_value()) {
            return (key_frame.gps.value() - new_gps).norm();
        }
    }
    return std::numeric_limits<double>::max();
}
void kiss_icp_ros::KeyFrameManager::update_key_frame_pose(const std::map<size_t, gtsam::Pose3> &new_poses) {

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &[key, pose]: new_poses) {
        if (key_frame_pose.contains(key)) {
            key_frame_pose.at(key) = pose;
        }
    }
}
double kiss_icp_ros::KeyFrameManager::get_angle_to_last_key(const gtsam::Rot3 &r) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (key_frames_.empty()) {
        return std::numeric_limits<double>::max();
    }
    size_t last_key_id = key_frames_.back().id;
    const auto &last_pose = key_frame_pose.at(last_key_id);

    auto bw = r.between(last_pose.rotation());
    return std::get<1>(bw.axisAngle());
}
double kiss_icp_ros::KeyFrameManager::get_distance_to_last_key(const gtsam::Point3 &p) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (key_frames_.empty()) {
        return std::numeric_limits<double>::max();
    }
    size_t last_key_id = key_frames_.back().id;
    const auto &last_pose = key_frame_pose.at(last_key_id);
    return (last_pose.translation() - p).norm();
}
std::vector<visualization_msgs::msg::Marker> kiss_icp_ros::KeyFrameManager::get_gps_markers(std_msgs::msg::Header header, std::string ns) const {
    std::lock_guard<std::mutex> lock(mutex_);

    visualization_msgs::msg::Marker gps_marker;
    gps_marker.header = header;
    gps_marker.ns = ns;
    gps_marker.id = 0;
    gps_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    gps_marker.action = visualization_msgs::msg::Marker::ADD;
    gps_marker.scale.x = 0.2;
    gps_marker.scale.y = 0.2;
    gps_marker.scale.z = 0.2;
    gps_marker.color.a = 1.0;// Don't forget to set the alpha!
    gps_marker.color.r = 0.0;
    gps_marker.color.g = 1.0;
    gps_marker.color.b = 0.0;


    visualization_msgs::msg::Marker key_frame_marker;
    key_frame_marker.header = header;
    key_frame_marker.ns = ns;
    key_frame_marker.id = 2;
    key_frame_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_frame_marker.action = visualization_msgs::msg::Marker::ADD;
    key_frame_marker.scale.x = 0.2;
    key_frame_marker.scale.y = 0.2;
    key_frame_marker.scale.z = 0.2;
    key_frame_marker.color.a = 1.0;// Don't forget to set the alpha!
    key_frame_marker.color.r = 0.0;
    key_frame_marker.color.g = 0.0;
    key_frame_marker.color.b = 1.0;


    visualization_msgs::msg::Marker gps_residual_marker;
    gps_residual_marker.header = header;
    gps_residual_marker.ns = ns;
    gps_residual_marker.id = 1;
    gps_residual_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    gps_residual_marker.action = visualization_msgs::msg::Marker::ADD;
    gps_residual_marker.scale.x = 0.1;
    gps_residual_marker.scale.y = 0.1;
    gps_residual_marker.scale.z = 0.1;
    gps_residual_marker.color.a = 1.0;// Don't forget to set the alpha!
    gps_residual_marker.color.r = 1.0;
    gps_residual_marker.color.g = 0.0;
    gps_residual_marker.color.b = 0.0;


    for (const auto &key_frame: key_frames_) {
        if (key_frame.gps.has_value()) {
            geometry_msgs::msg::Point gps_p;
            gps_p.x = key_frame.gps.value().x();
            gps_p.y = key_frame.gps.value().y();
            gps_p.z = key_frame.gps.value().z();

            gps_marker.points.push_back(gps_p);


            geometry_msgs::msg::Point gps_residual_p;
            gps_residual_p.x = key_frame.gps.value().x();
            gps_residual_p.y = key_frame.gps.value().y();
            gps_residual_p.z = key_frame.gps.value().z();
            gps_residual_marker.points.push_back(gps_residual_p);

            const auto &opt_pose = key_frame_pose.at(key_frame.id);
            geometry_msgs::msg::Point key_frame_point;
            key_frame_point.x = opt_pose.translation().x();
            key_frame_point.y = opt_pose.translation().y();
            key_frame_point.z = opt_pose.translation().z();
            gps_residual_marker.points.push_back(key_frame_point);
            key_frame_marker.points.push_back(key_frame_point);
        }
    }
    std::vector<visualization_msgs::msg::Marker> markers;

    markers.emplace_back(gps_marker);
    markers.emplace_back(gps_residual_marker);
    markers.emplace_back(key_frame_marker);

    return markers;
}
std::map<size_t, kiss_icp_ros::PointsWithOrigin> kiss_icp_ros::KeyFrameManager::get_map() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<size_t, kiss_icp_ros::PointsWithOrigin> r;
    for (const auto &key_frame: key_frames_) {
        const auto pose = key_frame_pose.at(key_frame.id);
        r[key_frame.id] = {pose, key_frame.points};
    }
    return r;
}
