//
// Created by jimmy on 23-8-10.
//
#include "utility.hpp"


class TransformFusion : public ParamServer {
public:
    std::mutex mtx;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuIncOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

    rclcpp::CallbackGroup::SharedPtr callbackGroupImuOdometry;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLaserOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    Eigen::Isometry3d lidarOdomAffine;


    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    double lidarOdomTime = -1;
    deque<nav_msgs::msg::Odometry> imuOdomQueue;

    TransformFusion(const rclcpp::NodeOptions &options) : ParamServer("lio_sam_transformFusion", options) {
        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        callbackGroupImuOdometry = create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupLaserOdometry = create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOdomOpt = rclcpp::SubscriptionOptions();
        imuOdomOpt.callback_group = callbackGroupImuOdometry;
        auto laserOdomOpt = rclcpp::SubscriptionOptions();
        laserOdomOpt.callback_group = callbackGroupLaserOdometry;


        subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>(
                "liorf/mapping/odometry", qos,
                std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1),
                laserOdomOpt);
        subImuIncOdometry = create_subscription<nav_msgs::msg::Odometry>(
                odomTopic + "_incremental", qos_imu,
                std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1),
                imuOdomOpt);

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, qos_imu);
    }


    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);
        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = stamp2Sec(odomMsg->header.stamp);
    }

    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest lidar odometry (at current IMU stamp)
        if (lidarOdomTime == -1) {
            //wait for lidar odometry
            return;
        }

        // remove odometry before current lidar odometry time
        while (!imuOdomQueue.empty()) {
            if (stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime) {
                imuOdomQueue.pop_front();
            } else {
                break;
            }
        }

        Eigen::Isometry3d imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Isometry3d imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Isometry3d imuOdomAffineInc = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Isometry3d imuOdomAffineLast = lidarOdomAffine * imuOdomAffineInc;
        geometry_msgs::msg::TransformStamped odom_to_laser = tf2::eigenToTransform(imuOdomAffineLast);

        // publish latest odometry
        nav_msgs::msg::Odometry latest_odom;
        latest_odom.header = odomMsg->header;
        latest_odom.child_frame_id = odomMsg->child_frame_id;
        latest_odom.pose.pose.position.x = odom_to_laser.transform.translation.x;
        latest_odom.pose.pose.position.y = odom_to_laser.transform.translation.y;
        latest_odom.pose.pose.position.z = odom_to_laser.transform.translation.z;
        latest_odom.pose.pose.orientation = odom_to_laser.transform.rotation;
        pubImuOdometry->publish(latest_odom);

        // publish tf
        tf2::Stamped<tf2::Transform> tCur;
        tf2::convert(odom_to_laser, tCur);
        if (lidarFrame != baselinkFrame) {
            tf2::Stamped<tf2::Transform> lidar2Baselink;
            try {
                tf2::fromMsg(tfBuffer->lookupTransform(
                                     lidarFrame, baselinkFrame, rclcpp::Time(0)),
                             lidar2Baselink);
            } catch (tf2::TransformException ex) {
                RCLCPP_ERROR(get_logger(), "lidar frame is not base_link frame , but the tf relation can not be found, due to %s", ex.what());
            }
            tf2::Stamped<tf2::Transform> tb(tCur * lidar2Baselink, tf2_ros::fromMsg(odomMsg->header.stamp), odometryFrame);
            tCur = tb;
        }
        geometry_msgs::msg::TransformStamped odom_to_base;
        tf2::convert(tCur, odom_to_base);
        odom_to_base.child_frame_id = odomMsg->child_frame_id;
        tfBroadcaster->sendTransform(odom_to_base);

    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::MultiThreadedExecutor e;
    auto TF = std::make_shared<TransformFusion>(options);
    e.add_node(TF);
    e.spin();
    rclcpp::shutdown();
    return 0;
}
