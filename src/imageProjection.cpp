#include "liorf/msg/cloud_info.hpp"
#include "utility.hpp"
#include <algorithm>
#include <array>
#include <pcl/filters/filter.h>

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

class ImageProjection : public ParamServer {
private:
    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<liorf::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;


    // imu data used for deskew
    std::array<double, queueLength> imuTime;
    std::array<double, queueLength> imuRotX;
    std::array<double, queueLength> imuRotY;
    std::array<double, queueLength> imuRotZ;

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;

    int deskewFlag;

    bool odomDeskewFlag;
    // odom data used for deskew
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    liorf::msg::CloudInfo cloudInfo;  // cloud info for output
    double timeScanCur;               // time of current scan
    double timeScanEnd;               // time of current scan end
    std_msgs::msg::Header cloudHeader;// latest cloud header


public:
    ImageProjection(const rclcpp::NodeOptions &options)
        : ParamServer("lio_sam_imageProjection", options), deskewFlag(0) {
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
        subOdom = create_subscription<nav_msgs::msg::Odometry>(
                odomTopic + "_incremental", qos_imu,
                std::bind(&ImageProjection::odometryHandler, this,
                          std::placeholders::_1),
                odomOpt);
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
                pointCloudTopic, qos_lidar,
                std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
                lidarOpt);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
                "liorf/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = create_publisher<liorf::msg::CloudInfo>(
                "liorf/deskew/cloud_info", qos);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());

        // todo: check if needed
        //    cloudInfo.start_ring_index.assign(N_SCAN, 0);
        //    cloudInfo.end_ring_index.assign(N_SCAN, 0);
        //    cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
        //    cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters() {
        laserCloudIn->clear();
        fullCloud->clear();

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i) {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection() {}

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " <<
        // imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg) {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
        if (!cachePointCloud(laserCloudMsg)) { return; }

        if (!deskewInfo()) { return; }

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &laserCloudMsg) {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);

        if (cloudQueue.size() <= 2) { return false; }

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        // convert cloud in my way
        pcl::PointCloud<PointXYZIRT>::Ptr tmpCloud(new pcl::PointCloud<PointXYZIRT>());
        pcl::fromROSMsg(currentCloudMsg, *tmpCloud);

        // remove nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*tmpCloud, *laserCloudIn, indices);

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

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = stamp2Sec(cloudHeader.stamp);
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;


        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0) {
            ringFlag = -1;
            for (int i = 0; i < (int) currentCloudMsg.fields.size(); ++i) {
                if (currentCloudMsg.fields[i].name == "ring") {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1) {
                //todo: calculate ring channel
                RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available, please configure your point cloud data!");
                rclcpp::shutdown();
            }
        }

        // check point time
        // todo: calculate point time
        if (deskewFlag == 0) {
            deskewFlag = -1;
            for (auto &field: currentCloudMsg.fields) {
                if (field.name == "time" || field.name == "t") {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1) {
                RCLCPP_WARN(get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }
        }

        return true;
    }

    bool deskewInfo() {
        std::scoped_lock lck{imuLock, odoLock};
        //        std::lock_guard<std::mutex> lock1(imuLock);
        //        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() ||
            stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||
            stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd) {
            RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    // find and save all imu msg within scan time, from imu_msg buffer
    void imuDeskewInfo() {
        cloudInfo.imu_available = false;


        // clean old IMU messages before current scan
        while (!imuQueue.empty()) {
            if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int) imuQueue.size(); ++i) {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

            if (currentImuTime > timeScanEnd + 0.01) { break; }// imu data too new, ignore for now


            if (imuType == ImuType::NINE_AXIS) {
                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur) {
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
                }
            }


            if (imuPointerCur == 0) {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] =
                    imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] =
                    imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] =
                    imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0) { return; }

        cloudInfo.imu_available = true;
    }


    // find and save all odom msg within scan time, from odom msg buffer
    void odomDeskewInfo() {
        cloudInfo.odom_available = false;
        static float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20;

        // clean old odometry messages before current scan
        while (!odomQueue.empty()) {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01) {
                odomQueue.pop_front();
            } else {
                break;
            }
        }

        if (odomQueue.empty()) {
            return;
        }

        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur) {
            return;
        }

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;
        for (int i = 0; i < (int) odomQueue.size(); ++i) {
            startOdomMsg = odomQueue[i];

            if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
                continue;
            else
                break;
        }

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw = yaw;
        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd) {
            return;
        }

        nav_msgs::msg::Odometry endOdomMsg;
        for (int i = 0; i < (int) odomQueue.size(); ++i) {
            endOdomMsg = odomQueue[i];

            if (stamp2Sec(endOdomMsg.header.stamp) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) !=
            int(round(endOdomMsg.pose.covariance[0]))) {
            return;
        }

        Eigen::Affine3f transBegin = pcl::getTransformation(
                startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y,
                startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(
                endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y,
                endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY,
                                          odomIncreZ, rollIncre, pitchIncre,
                                          yawIncre);

        odomDeskewFlag = true;
    }

    // get roll, pitch, and yaw change at specific time, from imu msg buffer
    void findRotation(double pointTime, float *rotXCur, float *rotYCur,
                      float *rotZCur) {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur) {
            if (pointTime < imuTime[imuPointerFront]) break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) /
                                (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) /
                               (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront +
                       imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront +
                       imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront +
                       imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur,
                      float *posZCur) {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional
        // deskew seems to have little benefits. Thus code below is commented.

        if (cloudInfo.odom_available == false || odomDeskewFlag == false)
            return;


        if (timeScanEnd - timeScanCur < 10E-3) {
            return;
        }

        float ratio = relTime / (timeScanEnd - timeScanCur);

        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime) {
        if (deskewFlag == -1 || cloudInfo.imu_available == false) {
            return *point;
        }

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true) {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                                        rotXCur, rotYCur, rotZCur))
                                        .inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(
                posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y +
                     transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y +
                     transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y +
                     transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud() {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i) {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange) { continue; }

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN) { continue; }

            if (rowIdn % downsampleRate != 0) { continue; }// hard core down sampling

            if (i % point_filter_num != 0) { continue; }// hard core down sampling

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            fullCloud->push_back(thisPoint);
        }
    }

    void publishClouds() {
        cloudInfo.header = cloudHeader;
        auto fullCloud_msg = publishCloud(pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_deskewed = std::move(fullCloud_msg);
        pubLaserCloudInfo->publish(cloudInfo);
    }
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
