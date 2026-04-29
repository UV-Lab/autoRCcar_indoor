#ifndef F27DAF94_5850_4F9C_9FC9_8916EDE10A2E
#define F27DAF94_5850_4F9C_9FC9_8916EDE10A2E

#include <memory>

#include <Eigen/Eigen>
// #include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace msf_loc {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
// using PointWithTimestampT = pcl::pointxyzi

class LidarCloudData {
public:
    double timestamp;
    CloudT::Ptr cloud_ptr;
};

class ImuData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time_stamp_sec;
    double dt;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};

class ImuSensorParams {
public:
    using SharedPtr = std::shared_ptr<ImuSensorParams>;
    double gravity = 9.80511;
    Eigen::Vector3d gravity_in_map_frame;
    double accel_sigma = 1e-3;
    double gyro_sigma = 1e-3;
    double accel_bias_sigma = 1e-5;
    double gyro_bias_sigma = 1e-5;
    // double integration_cov = 1e-8;
};

class PreintegrationParams {
public:
    using SharedPtr = std::shared_ptr<PreintegrationParams>;
    double correction_noise_pos = 0.05;
    double correction_noise_rot = 0.1;
    double integration_cov = 1e-8;
    double prior_pose_noise = 1e-2;
    double prior_vel_noise = 1e-1;
    double prior_bias_noise = 1e-3;
};

// class BodyPose3D {
// public:
//     double timestamp;
//     Sophus::SE3d pose;
//     // Eigen::Vector3d trans;
//     // Eigen::Quaterniond quat;
// };

using BodyVel3D = Eigen::Vector3d;
class ImuBodyBias {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d bias_accel;
    Eigen::Vector3d bias_gyro;
};


}// namespace msf_loc

#endif /* F27DAF94_5850_4F9C_9FC9_8916EDE10A2E */
