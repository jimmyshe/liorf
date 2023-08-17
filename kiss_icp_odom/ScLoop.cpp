//
// Created by jimmy on 23-8-17.
//

#include "ScLoop.h"
void kiss_icp_ros::ScLoop::CalculateLoopClosure(size_t laser_id, std::vector<Eigen::Vector3d> points) {
    static int sc_index = 0;
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::PointCloud<SCPointType>::Ptr cloud(new pcl::PointCloud<SCPointType>);
    for (const auto &p: points) {
        SCPointType pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        cloud->push_back(pt);
    }
    sc_manager_.makeAndSaveScancontextAndKeys(*cloud);
    key_to_sc_key_dict[laser_id] = sc_index;
    sc_key_to_cloud_dict[sc_index] = cloud;
    sc_to_lidar_key_dict[sc_index] = laser_id;
    int current_sc_index = sc_index;
    sc_index++;

    const auto [related_sc_index, yaw_diff] = sc_manager_.detectLoopClosureID();

    if (related_sc_index == -1) {
        RCLCPP_INFO(rclcpp::get_logger("scloop"), "No loop closure found");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("scloop"), "Loop closure found %d to %d", current_sc_index, related_sc_index);

    Eigen::Isometry3d current2related_original;
    {
        auto k2p = key_to_pose_dict.synchronize();
        auto related_pose = k2p->at(sc_to_lidar_key_dict.at(related_sc_index));
        auto current_pose = k2p->at(sc_to_lidar_key_dict.at(current_sc_index));
        current2related_original = current_pose.inverse() * related_pose;
    }

    const auto &current_cloud = cloud;
    auto related_cloud = sc_key_to_cloud_dict.at(related_sc_index);


    constexpr double historyKeyframeSearchRadius = 15;
    constexpr double historyKeyframeFitnessScore = 0.3;

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(current_cloud);
    icp.setInputTarget(related_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result, current2related_original.matrix().cast<float>());

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
        RCLCPP_INFO(rclcpp::get_logger("scloop"), "ICP has not converged, fitness score is %f", icp.getFitnessScore());
        return;
    }


    Eigen::Matrix4d final = icp.getFinalTransformation().cast<double>();
    Eigen::Isometry3d correction(final);
    RCLCPP_INFO(rclcpp::get_logger("scloop"), "ICP has converged, fitness score is %f", icp.getFitnessScore());

    LoopClosureData data;
    data.from_id = sc_to_lidar_key_dict.at(current_sc_index);
    data.to_id = sc_to_lidar_key_dict.at(related_sc_index);
    data.transform = correction;
    data.fitness = icp.getFitnessScore();
    {
        auto lock = loop_closure_data_.synchronize();
        lock->push_back(data);
    }
}
std::vector<visualization_msgs::msg::Marker> kiss_icp_ros::ScLoop::get_loop_closure_markers(std_msgs::msg::Header header, std::string ns) {

    auto closure_date_copy = get_all_closure();
    auto key_to_pose_dict_copy = key_to_pose_dict.get();

    visualization_msgs::msg::Marker closure_marker;
    closure_marker.header = header;
    closure_marker.ns = ns;
    closure_marker.id = 0;
    closure_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    closure_marker.action = visualization_msgs::msg::Marker::ADD;
    closure_marker.scale.x = 0.1;
    closure_marker.scale.y = 0.1;
    closure_marker.scale.z = 0.1;
    closure_marker.color = get_loop_closure_color();

    visualization_msgs::msg::Marker residual_marker;
    residual_marker.header = header;
    residual_marker.ns = ns;
    residual_marker.id = 1;
    residual_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    residual_marker.action = visualization_msgs::msg::Marker::ADD;
    residual_marker.scale.x = 0.1;
    residual_marker.scale.y = 0.1;
    residual_marker.scale.z = 0.1;
    residual_marker.color = get_loop_closure_color();


    for (const auto &item: closure_date_copy) {
        auto from_pose = key_to_pose_dict_copy.at(item.from_id);
        auto to_pose = key_to_pose_dict_copy.at(item.to_id);
        auto match_pose = item.transform * from_pose;

        geometry_msgs::msg::Point from_pose_point;
        from_pose_point.x = from_pose.translation().x();
        from_pose_point.y = from_pose.translation().y();
        from_pose_point.z = from_pose.translation().z();

        geometry_msgs::msg::Point to_pose_point;
        to_pose_point.x = to_pose.translation().x();
        to_pose_point.y = to_pose.translation().y();
        to_pose_point.z = to_pose.translation().z();

        geometry_msgs::msg::Point match_pose_point;
        match_pose_point.x = match_pose.translation().x();
        match_pose_point.y = match_pose.translation().y();
        match_pose_point.z = match_pose.translation().z();

        closure_marker.points.push_back(from_pose_point);
        closure_marker.points.push_back(match_pose_point);

        residual_marker.points.push_back(match_pose_point);
        residual_marker.points.push_back(to_pose_point);
    }


    std::vector<visualization_msgs::msg::Marker> markers;
    markers.push_back(closure_marker);
    markers.push_back(residual_marker);


    return markers;
}
