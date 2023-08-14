#include "MsgBuffer.h"
#include "liorf/msg/cloud_info.hpp"
#include "utility.hpp"
#include <algorithm>
#include <array>
#include <optional>
#include <pcl/filters/filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

// <!-- liorf_localization_yjz_lucky_boy -->
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
        VelodynePointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                intensity)(uint16_t, ring,
                                                           ring)(float, time, time))

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
        OusterPointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(
                uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

struct RobosensePointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
        RobosensePointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                uint16_t, ring, ring)(double, timestamp, timestamp))

// mulran datasets
struct MulranPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
        MulranPointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                intensity)(uint32_t, t,
                                                           t)(int, ring, ring))

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;
using namespace std::chrono_literals;

/*!
 * \brief ImageProjection class
 *  deskew the point cloud and pub as CloudInfo
 */
class ImageProjection : public ParamServer {
private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDeskewedCloud;
    rclcpp::Publisher<liorf::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    MsgBuffer<sensor_msgs::msg::Imu> imuMsgBuffer;
    std::mutex imuLock;


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuIncOdometry;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    MsgBuffer<nav_msgs::msg::Odometry> odomMsgBuffer;
    std::mutex odoLock;







public:
    ImageProjection(const rclcpp::NodeOptions &options)
        : ParamServer("lio_sam_imageProjection", options),imuMsgBuffer(queueLength), odomMsgBuffer(queueLength) {


        tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Create the timer interface before call to waitForTransform,
        // to avoid a tf2_ros::CreateTimerInterfaceException exception
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                this->get_node_base_interface(),
                this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        callbackGroupLidar =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupImu =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
        lidarOpt.callback_group = callbackGroupLidar;
        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subImu = create_subscription<sensor_msgs::msg::Imu>(
                imuTopic, qos_imu,
                std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1),
                imuOpt);
        subImuIncOdometry = create_subscription<nav_msgs::msg::Odometry>(
                odomTopic + "_incremental", qos_imu,
                std::bind(&ImageProjection::odometryHandler, this,
                          std::placeholders::_1),
                odomOpt);
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
                pointCloudTopic, qos_lidar,
                std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
                lidarOpt);

        pubDeskewedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
                "liorf/cloud_deskewed", 1);
        pubLaserCloudInfo = create_publisher<liorf::msg::CloudInfo>(
                "liorf/cloud_info", qos);



        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }





    ~ImageProjection() {}

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
        std::lock_guard<std::mutex> lock1(imuLock);
        int64_t time = stamp2NanoSec(imuMsg->header.stamp);
        imuMsgBuffer.put_in(time, *imuMsg);
        RCLCPP_DEBUG(this->get_logger(), "imuHandler put %ld in buffer", time);
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg) {
        std::lock_guard<std::mutex> lock2(odoLock);
        int64_t time = stamp2NanoSec(odometryMsg->header.stamp);
        odomMsgBuffer.put_in(time, *odometryMsg);
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {

        auto raw_cloud_opt = cachedReturn(laserCloudMsg);

        if (not raw_cloud_opt.has_value()) {
            return;
        }

        pcl::PointCloud<PointXYZIRT>::Ptr raw_laser = ConvertPointCloud(raw_cloud_opt.value());

        if (raw_laser->empty()) {
            RCLCPP_WARN(get_logger(), "cloudHandler: empty cloud");
            return;
        }

        int64_t laser_time = stamp2NanoSec(raw_cloud_opt.value().header.stamp);
        int64_t laser_end_time = laser_time + int64_t(raw_laser->points.back().time * 1e9);
        constexpr int64_t epsilon_t = std::chrono::duration_cast<std::chrono::nanoseconds>(10ms).count();


        liorf::msg::CloudInfo cloudInfo_to_be_pubbed;
        cloudInfo_to_be_pubbed.header.frame_id = baselinkFrame;
        cloudInfo_to_be_pubbed.header.stamp = laserCloudMsg->header.stamp;


        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data_vec;
        {
            std::lock_guard lock(imuLock);
            imu_data_vec = imuMsgBuffer.get_between(laser_time - epsilon_t, laser_end_time + epsilon_t);
        }
        RCLCPP_INFO(get_logger(), "imu data size: %ld", imu_data_vec.size());
        cloudInfo_to_be_pubbed.imu_available = !imu_data_vec.empty();
        if(cloudInfo_to_be_pubbed.imu_available){
            if (imuType == ImuType::NINE_AXIS) {
                // get roll, pitch, and yaw estimation for this scan
                const sensor_msgs::msg::Imu& thisImuMsg = *imu_data_vec.front();
                imuRPY2rosRPY(thisImuMsg, &cloudInfo_to_be_pubbed.imu_roll_init, &cloudInfo_to_be_pubbed.imu_pitch_init, &cloudInfo_to_be_pubbed.imu_yaw_init);
            }
        }



        std::vector<nav_msgs::msg::Odometry::ConstSharedPtr> odom_data_vec;
        std::optional<nav_msgs::msg::Odometry::ConstSharedPtr> start_odom_opt;

        {
            std::lock_guard lock(odoLock);
            odom_data_vec = odomMsgBuffer.get_between(laser_time - epsilon_t, laser_end_time + epsilon_t);
            start_odom_opt = odomMsgBuffer.get_msg_cloest(laser_time,10ms);
        }
        RCLCPP_INFO(get_logger(), "odom data size: %ld", odom_data_vec.size());
        cloudInfo_to_be_pubbed.odom_available = start_odom_opt.has_value();
        if (cloudInfo_to_be_pubbed.odom_available){
            // get start odometry at the beinning of the scan
            tf2::Quaternion orientation;
            tf2::fromMsg(start_odom_opt.value()->pose.pose.orientation, orientation);

            double roll, pitch, yaw;
            tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

            // Initial guess used in mapOptimization
            cloudInfo_to_be_pubbed.initial_guess_x = start_odom_opt.value()->pose.pose.position.x;
            cloudInfo_to_be_pubbed.initial_guess_y = start_odom_opt.value()->pose.pose.position.y;
            cloudInfo_to_be_pubbed.initial_guess_z = start_odom_opt.value()->pose.pose.position.z;
            cloudInfo_to_be_pubbed.initial_guess_roll = roll;
            cloudInfo_to_be_pubbed.initial_guess_pitch = pitch;
            cloudInfo_to_be_pubbed.initial_guess_yaw = yaw;

        }














        pcl::PointCloud<PointType>::Ptr fullCloud_to_pubbed(new pcl::PointCloud<PointType>());


        size_t cloudSize = raw_laser->points.size();
        // range image projection
        for (size_t i = 0; i < cloudSize; ++i) {
            PointType thisPoint;
            thisPoint.x = raw_laser->points[i].x;
            thisPoint.y = raw_laser->points[i].y;
            thisPoint.z = raw_laser->points[i].z;
            thisPoint.intensity = raw_laser->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange) { continue; }

            int rowIdn = raw_laser->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN) { continue; }

            if (rowIdn % downsampleRate != 0) { continue; }// hard core down sampling

            if (i % point_filter_num != 0) { continue; }// hard core down sampling

//            thisPoint = deskewPoint(&thisPoint, raw_laser->points[i].time); // todo:  deskewPoint according to imu and odom

            fullCloud_to_pubbed->push_back(thisPoint);
        }




        auto fullCloud_msg = publishCloud(pubDeskewedCloud, fullCloud_to_pubbed, cloudInfo_to_be_pubbed.header.stamp, baselinkFrame);
        cloudInfo_to_be_pubbed.cloud_deskewed = std::move(fullCloud_msg);
        pubLaserCloudInfo->publish(cloudInfo_to_be_pubbed);



    }


    std::optional<sensor_msgs::msg::PointCloud2> cachedReturn(const sensor_msgs::msg::PointCloud2::SharedPtr &laserCloudMsg) {
        constexpr size_t maxDelay = 2;
        static std::deque<sensor_msgs::msg::PointCloud2> delay_cloud_q{};
        delay_cloud_q.push_back(*laserCloudMsg);
        if (delay_cloud_q.size() > maxDelay) {
            auto ret = delay_cloud_q.front();
            delay_cloud_q.pop_front();
            return ret;
        }
        return {};
    }



    pcl::PointCloud<PointXYZIRT>::Ptr ConvertPointCloud(const sensor_msgs::msg::PointCloud2 &laserCloudMsg) {

        try {
            // cache point cloud
            auto lidar2base = tf_buffer_->lookupTransform(laserCloudMsg.header.frame_id, baselinkFrame, tf2::TimePointZero);// if the tf is not static, we should look at the frame time
            sensor_msgs::msg::PointCloud2 cloudAtBase;
            tf2::doTransform(laserCloudMsg, cloudAtBase, lidar2base);

            // convert cloud in my way
            pcl::PointCloud<PointXYZIRT>::Ptr tmpCloud(new pcl::PointCloud<PointXYZIRT>());
            pcl::fromROSMsg(laserCloudMsg, *tmpCloud);

            // remove nan points
            pcl::PointCloud<PointXYZIRT>::Ptr laserCloudWithOutNan(new pcl::PointCloud<PointXYZIRT>());
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*tmpCloud, *laserCloudWithOutNan, indices);


            //        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX) {
            //            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
            //        } else if (sensor == SensorType::OUSTER) {
            //            // Convert to Velodyne format
            //            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            //            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            //            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            //            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++) {
            //                auto &src = tmpOusterCloudIn->points[i];
            //                auto &dst = laserCloudIn->points[i];
            //                dst.x = src.x;
            //                dst.y = src.y;
            //                dst.z = src.z;
            //                dst.intensity = src.intensity;
            //                dst.ring = src.ring;
            //                dst.time = src.t * 1e-9f;
            //            }
            //        }// <!-- liorf_yjz_lucky_boy -->
            //        else if (sensor == SensorType::MULRAN) {
            //            // Convert to Velodyne format
            //            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            //            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            //            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            //            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++) {
            //                auto &src = tmpMulranCloudIn->points[i];
            //                auto &dst = laserCloudIn->points[i];
            //                dst.x = src.x;
            //                dst.y = src.y;
            //                dst.z = src.z;
            //                dst.intensity = src.intensity;
            //                dst.ring = src.ring;
            //                dst.time = static_cast<float>(src.t);
            //            }
            //        }// <!-- liorf_yjz_lucky_boy -->
            //        else if (sensor == SensorType::ROBOSENSE) {
            //            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(
            //                    new pcl::PointCloud<RobosensePointXYZIRT>());
            //            // Convert to robosense format
            //            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            //            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            //            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;
            //
            //            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            //            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
            //                auto &src = tmpRobosenseCloudIn->points[i];
            //                auto &dst = laserCloudIn->points[i];
            //                dst.x = src.x;
            //                dst.y = src.y;
            //                dst.z = src.z;
            //                dst.intensity = src.intensity;
            //                dst.ring = src.ring;
            //                dst.time = src.timestamp - start_stamptime;
            //            }
            //        } else {
            //            RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(sensor));
            //            rclcpp::shutdown();
            //        }

            //        if (laserCloudIn->is_dense == false) {
            //            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            //            rclcpp::shutdown();
            //        }

            return laserCloudWithOutNan;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "lidar 2 base tf is not available %s", ex.what());
            rclcpp::shutdown();
            std::terminate();
        }
    }



    // get roll, pitch, and yaw change at specific time, from imu msg buffer
//    void findRotation(double pointTime, float *rotXCur, float *rotYCur,
//                      float *rotZCur) {
//        *rotXCur = 0;
//        *rotYCur = 0;
//        *rotZCur = 0;
//
//        int imuPointerFront = 0;
//        while (imuPointerFront < imuPointerCur) {
//            if (pointTime < imuTime[imuPointerFront]) break;
//            ++imuPointerFront;
//        }
//
//        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
//            *rotXCur = imuRotX[imuPointerFront];
//            *rotYCur = imuRotY[imuPointerFront];
//            *rotZCur = imuRotZ[imuPointerFront];
//        } else {
//            int imuPointerBack = imuPointerFront - 1;
//            double ratioFront = (pointTime - imuTime[imuPointerBack]) /
//                                (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
//            double ratioBack = (imuTime[imuPointerFront] - pointTime) /
//                               (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
//            *rotXCur = imuRotX[imuPointerFront] * ratioFront +
//                       imuRotX[imuPointerBack] * ratioBack;
//            *rotYCur = imuRotY[imuPointerFront] * ratioFront +
//                       imuRotY[imuPointerBack] * ratioBack;
//            *rotZCur = imuRotZ[imuPointerFront] * ratioFront +
//                       imuRotZ[imuPointerBack] * ratioBack;
//        }
//    }

//    void findPosition(double relTime, float *posXCur, float *posYCur,
//                      float *posZCur) {
//        *posXCur = 0;
//        *posYCur = 0;
//        *posZCur = 0;
//
//        // If the sensor moves relatively slow, like walking speed, positional
//        // deskew seems to have little benefits. Thus code below is commented.
//
//        if (cloudInfo.odom_available == false || odomDeskewFlag == false)
//            return;
//
//
//        if (timeScanEnd - timeScanCur < 10E-3) {
//            return;
//        }
//
//        float ratio = relTime / (timeScanEnd - timeScanCur);
//
//        *posXCur = ratio * odomIncreX;
//        *posYCur = ratio * odomIncreY;
//        *posZCur = ratio * odomIncreZ;
//    }

//    PointType deskewPoint(PointType *point, double relTime) {
//        if (deskewFlag == -1 || cloudInfo.imu_available == false) {
//            return *point;
//        }
//
//        double pointTime = timeScanCur + relTime;
//
//        float rotXCur, rotYCur, rotZCur;
//        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);
//
//        float posXCur, posYCur, posZCur;
//        findPosition(relTime, &posXCur, &posYCur, &posZCur);
//
//        if (firstPointFlag == true) {
//            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
//                                                        rotXCur, rotYCur, rotZCur))
//                                        .inverse();
//            firstPointFlag = false;
//        }
//
//        // transform points to start
//        Eigen::Affine3f transFinal = pcl::getTransformation(
//                posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
//        Eigen::Affine3f transBt = transStartInverse * transFinal;
//
//        PointType newPoint;
//        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y +
//                     transBt(0, 2) * point->z + transBt(0, 3);
//        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y +
//                     transBt(1, 2) * point->z + transBt(1, 3);
//        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y +
//                     transBt(2, 2) * point->z + transBt(2, 3);
//        newPoint.intensity = point->intensity;
//
//        return newPoint;
//    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImageProjection>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
