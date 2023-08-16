//
// Created by jimmy on 23-8-15.
//
#pragma  once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <nav_msgs/msg/path.hpp>


inline gtsam::Pose3 sophusSE3TogtsamPose3(const Sophus::SE3d& pose) {

    return gtsam::Pose3(
            pose.matrix()
            );

}

inline Sophus::SE3d gtsamPose3toSouphusSE3(const gtsam::Pose3& pose){
    return Sophus::SE3d(pose.matrix());
}



inline nav_msgs::msg::Path gtsamValues2Path(const gtsam::Values& vs,const std_msgs::msg::Header& header ) {

    nav_msgs::msg::Path p;
    p.header = header;
    for (const auto &key: vs.keys()){
        geometry_msgs::msg::PoseStamped pose_msg;
        auto pose =   vs.at<gtsam::Pose3>(key);
        Eigen::Quaterniond q(pose.rotation().matrix());

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        pose_msg.pose.position.x = pose.x();
        pose_msg.pose.position.y = pose.y();
        pose_msg.pose.position.z = pose.z();
        pose_msg.header = header;
        p.poses.push_back(pose_msg);
    }
    return p;

}
