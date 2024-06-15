// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include <Eigen/Core>
#include <Python.h>
#include <csignal>
#include <fstream>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <livox_ros_driver/CustomMsg.h>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>

// gstam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// gnss
#include "GNSS_Processing.hpp"
#include "sensor_msgs/NavSatFix.h"

// save map
#include "fast_lio_sam/save_map.h"
#include "fast_lio_sam/save_pose.h"

// save data in kitti format
#include <fstream>
#include <iomanip>
#include <sstream>

// using namespace gtsam;

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN],
    s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0}; // residuals, square sum of point to plane distance
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double filter_x_lower = 0, filter_x_upper = 0, filter_y_lower = 0, filter_y_upper = 0, filter_z_lower = 0,
       filter_z_upper = 0;
double icp_noise_boost_ratio = 1;
double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1,
    pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool is_curr_degraded = false, is_benefit_from_lc = false;
bool is_front_backend_tightly_coupled = false;
std::vector<int> degraded_poses(300, -1);
double t_isam_start, t_isam_end;
int idx_degraded_pose = 0;
double HTH_eigen_val_thres;

vector<vector<int>> pointSearchInd_surf;
vector<BoxPointType> cub_needrm; // points to be moved out of ikd-tree
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;               // record lidar time
deque<PointCloudXYZI::Ptr> lidar_buffer; // record lidar feature point cloud, or downsampled point cloud
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr
    feats_down_body(new PointCloudXYZI()); // single point cloud after deskew and downsampling, LiDAR frame
PointCloudXYZI::Ptr
    feats_down_world(new PointCloudXYZI()); // single point cloud after deskew and downsampling, world frame
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1)); // norm vector of feature points, world frame
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray; // points to be removed from the map

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d); // t, R, lidar to imu (imu = R * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf; // state, noise dimension, input
state_ikfom state_point;
vect3 pos_lid; // position of lidar in world frame

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
nav_msgs::Odometry loopConstrain;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

/*back end*/
vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());         // key frame position
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); // key frame pose
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D(
    new pcl::PointCloud<PointTypePose>()); //  store the poses that are not optimized by isam
pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

// voxel filter paprams
float odometrySurfLeafSize;
float mappingCornerLeafSize;
float mappingSurfLeafSize;

float z_tollerance;
float rotation_tollerance;

// CPU Params
int numberOfCores = 4;
double mappingProcessInterval;

/*loop clousre*/
bool startFlag = true;
bool loopClosureEnableFlag;
float loopClosureFrequency; // frequency of loop closure, better not too high
int surroundingKeyframeSize;
float historyKeyframeSearchRadius;   // search radius of kdtree for loop closure
float historyKeyframeSearchTimeDiff; // time difference of current frame and  loop closure frame should be larger than
int historyKeyframeSearchNum;        // when loop closure frame found, the number of keyframes to form a submap for icp
float historyKeyframeFitnessScore;   // icp threshold for matching
bool potentialLoopFlag = false;

ros::Publisher pubHistoryKeyFrames;
ros::Publisher pubIcpKeyFrames;
ros::Publisher pubRecentKeyFrames;
ros::Publisher pubRecentKeyFrame;
ros::Publisher pubCloudRegisteredRaw;
ros::Publisher pubLoopConstraintEdge;

bool aLoopIsClosed = false;
map<int, int> loopIndexContainer; // from new to old
vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
deque<std_msgs::Float64MultiArray> loopInfoVec;

nav_msgs::Path globalPath;

pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());

pcl::VoxelGrid<PointType> downSizeFilterCorner;
// pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterICP;
pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

float transformTobeMapped[10]; // current pose world frame

std::mutex mtx;
std::mutex mtxLoopInfo;

// Surrounding map
float surroundingkeyframeAddingDistThreshold; // threshold for selecting key frame
float surroundingkeyframeAddingAngleThreshold;
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

// gtsam
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::Values optimizedEstimate;
gtsam::ISAM2* isam;
gtsam::Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;

ros::Publisher pubLaserCloudSurround;
ros::Publisher pubOptimizedGlobalMap;

bool reconstructKdTree = false;
int updateKdtreeCount = 0;
bool visulize_IkdtreeMap = false; //  visual iktree submap

// gnss
double last_timestamp_gnss = -1.0;
deque<nav_msgs::Odometry> gnss_buffer;
geometry_msgs::PoseStamped msg_gnss_pose;
string gnss_topic;
bool useImuHeadingInitialization;
bool useGpsElevation; // if use GNSS elevation
float gpsCovThreshold;
float poseCovThreshold; // pose covariance threshold from isam2

M3D Gnss_R_wrt_Lidar(Eye3d); // gnss and imu extinsic
V3D Gnss_T_wrt_Lidar(Zero3d);
bool gnss_inited = false;
shared_ptr<GnssProcess> p_gnss(new GnssProcess());
GnssProcess gnss_data;
ros::Publisher pubGnssPath;
nav_msgs::Path gps_path;
vector<double> extrinT_Gnss2Lidar(3, 0.0);
vector<double> extrinR_Gnss2Lidar(9, 0.0);

// global map visualization radius
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

// saveMap
ros::ServiceServer srvSaveMap;
ros::ServiceServer srvSavePose;
bool savePCD;
string savePCDDirectory;

void updatePath(const PointTypePose& pose_in) {
    string odometryFrame = "camera_init";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    pose_stamped.pose.orientation.w = pose_in.qw;
    pose_stamped.pose.orientation.x = pose_in.qx;
    pose_stamped.pose.orientation.y = pose_in.qy;
    pose_stamped.pose.orientation.z = pose_in.qz;

    globalPath.poses.push_back(pose_stamped);
}

/**
 * transform point cloud, considering extrinsic
 */
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                                    PointTypePose* transformIn) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Isometry3d T_lidar_b(state_point.offset_R_L_I); // lidar to body
    T_lidar_b.pretranslate(state_point.offset_T_L_I);
    Eigen::Quaternionf q(transformIn->qw, transformIn->qx, transformIn->qy, transformIn->qz);
    Eigen::Vector3f t(transformIn->x, transformIn->y, transformIn->z);
    Eigen::Affine3f T_b_w_ = Eigen::Affine3f::Identity();
    T_b_w_.translate(t);
    T_b_w_.rotate(q);
    Eigen::Isometry3d T_b_w; //   body to world
    T_b_w.matrix() = T_b_w_.matrix().cast<double>();

    Eigen::Isometry3d T_lidar_w = T_b_w * T_lidar_b; // lidar to world

    Eigen::Isometry3d transCur = T_lidar_w;

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

/**
 * pose format change
 */
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
    gtsam::Quaternion q(thisPoint.qw, thisPoint.qx, thisPoint.qy, thisPoint.qz);
    return gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    // use quaternion instead of euler angle
    // return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
    //                     gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

/**
 * pose format change
 */
gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
    gtsam::Quaternion q(transformIn[9], transformIn[6], transformIn[7], transformIn[8]);
    return gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    // return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
    //                     gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

gtsam::Pose3 trans2gtsamPoseInit(float transformIn[]) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

/**
 * pose format change
 */
Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) {
    Eigen::Quaternionf q(thisPoint.qw, thisPoint.qx, thisPoint.qy, thisPoint.qz);
    Eigen::Vector3f t(thisPoint.x, thisPoint.y, thisPoint.z);
    Eigen::Affine3f T_ = Eigen::Affine3f::Identity();
    T_.translate(t);
    T_.rotate(q);
    return T_;
    // return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch,
    // thisPoint.yaw);
}

/**
 * pose format change
 */
Eigen::Affine3f trans2Affine3f(float transformIn[]) {
    Eigen::Quaternionf q(transformIn[9], transformIn[6], transformIn[7], transformIn[8]);
    Eigen::Vector3f t(transformIn[3], transformIn[4], transformIn[5]);
    Eigen::Affine3f T_ = Eigen::Affine3f::Identity();
    T_.translate(t);
    T_.rotate(q);
    return T_;
    // return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1],
    // transformIn[2]);
}

/**
 * pose format change
 */
PointTypePose trans2PointTypePose(float transformIn[]) {
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    thisPose6D.qw = transformIn[9];
    thisPose6D.qx = transformIn[6];
    thisPose6D.qy = transformIn[7];
    thisPose6D.qz = transformIn[8];
    return thisPose6D;
}

sensor_msgs::PointCloud2 publishCloud(ros::Publisher* thisPub, pcl::PointCloud<PointType>::Ptr thisCloud,
                                      ros::Time thisStamp, std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

float pointDistance(PointType p) { return sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

/**
 * initialization
 */
void allocateMemory() {
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudOri.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    for (int i = 0; i < 6; ++i) {
        transformTobeMapped[i] = 0;
    }
}

//  eulerAngle 2 Quaterniond
Eigen::Quaterniond EulerToQuat(float roll_, float pitch_, float yaw_) {
    Eigen::Quaterniond q; // q and -q are the same
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll;
    q.normalize();
    return q;
}

void getCurPose(state_ikfom cur_state) {
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2, 1, 0); //  yaw pitch roll rad

    transformTobeMapped[0] = eulerAngle(2);    // roll when use eulerAngles(2,1,0), sequence is  ypr
    transformTobeMapped[1] = eulerAngle(1);    // pitch
    transformTobeMapped[2] = eulerAngle(0);    // yaw
    transformTobeMapped[3] = cur_state.pos(0); // x
    transformTobeMapped[4] = cur_state.pos(1); // y
    transformTobeMapped[5] = cur_state.pos(2); // z
    Eigen::Quaternionf q(cur_state.rot.w(), cur_state.rot.x(), cur_state.rot.y(), cur_state.rot.z());
    q.normalize();
    transformTobeMapped[6] = q.x();
    transformTobeMapped[7] = q.y();
    transformTobeMapped[8] = q.z();
    transformTobeMapped[9] = q.w();
}

/**
 * show loop closure edge on rivz
 */
void visualizeLoopClosure() {
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";

    if (loopIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // edge
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}

/**
 * calculate the pose difference, if larger than threshold, set as keyframe
 */
bool saveFrame() {
    if (cloudKeyPoses3D->points.empty())
        return true;

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);

    // pose difference
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll) < surroundingkeyframeAddingAngleThreshold && 
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;
    return true;
}

/**
 * add fast lio factor
 */
void addOdomFactor() {
    if (cloudKeyPoses3D->points.empty()) {
        // the first frame is initialized as priori infomation
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
        // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2, 1e-2, M_PI*M_PI, 1e8,
        // 1e8, 1e8
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    } else {
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); // pre
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                   // cur
        // prev key frame id, curr key frame id, pose from pre to curr
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
                                                          poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

/**
 * add loop closure fator
 */
void addLoopFactor() {
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
        int indexFrom = loopIndexQueue[i].first; // cur
        int indexTo = loopIndexQueue[i].second;  // pre
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

/**
 * add GNSS factor
 */
void addGPSFactor() {
    if (gnss_buffer.empty())
        return;
    if (cloudKeyPoses3D->points.empty())
        return;
    else {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }
    // if the covariance of pose is small, GNSS position is not necessary
    if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
        return;
    static PointType lastGPSPoint;
    while (!gnss_buffer.empty()) {

        if (gnss_buffer.front().header.stamp.toSec() < lidar_end_time - 0.05) {
            gnss_buffer.pop_front();
        }

        else if (gnss_buffer.front().header.stamp.toSec() > lidar_end_time + 0.05) {
            break;
        } else {
            nav_msgs::Odometry thisGPS = gnss_buffer.front();
            gnss_buffer.pop_front();
            // check the covariance
            float noise_x = thisGPS.pose.covariance[0];
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];
            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                continue;
            float gps_x = thisGPS.pose.pose.position.x;
            float gps_y = thisGPS.pose.pose.position.y;
            float gps_z = thisGPS.pose.pose.position.z;
            if (!useGpsElevation) {
                gps_z = transformTobeMapped[5];
                noise_z = 0.01;
            }

            // 0 0 0 are invalid data
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                continue;
            // add GNSS every 5 meters
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                continue;
            else
                lastGPSPoint = curGPSPoint;
            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);
            aLoopIsClosed = true;
            ROS_INFO("GPS Factor Added");
            break;
        }
    }
}

void saveKeyFramesAndFactor() {
    // check if current frame is key frame according to distance
    if (saveFrame() == false)
        return;
    // add odometry factor (relative pose in body frame) from fast-lio
    addOdomFactor();
    addGPSFactor();
    t_isam_start = omp_get_wtime();
    addLoopFactor(); // add loop closure, based on rs-search

    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    if (aLoopIsClosed == true) // if loop closure is detected, update more to get better linearization
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        is_benefit_from_lc = true;
    }
    t_isam_end = omp_get_wtime();
    // clear the factor graph, but isam has stored the infomation already
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    state_ikfom state_updated = kf.get_x(); // get current pose, without correction

    // update the state
    isamCurrentEstimate = isam->calculateBestEstimate();
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
    Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(),
                        latestEstimate.translation().z());
    gtsam::Quaternion q(latestEstimate.rotation().toQuaternion());
    q.normalize();

    state_updated.pos = pos;
    state_updated.rot = q;
    if (q.w() * state_updated.rot.w() <= 0 && q.x() * state_updated.rot.x() <= 0 &&
        q.y() * state_updated.rot.y() <= 0 && q.z() * state_updated.rot.z() <= 0) {
        state_updated.rot = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
    } else {
        state_updated.rot = q;
    }
    state_point = state_updated; // update state_point for visualization

    if (is_front_backend_tightly_coupled)
        kf.change_x(state_updated); // if the frontend and backend are tightly coupled, then update the frontend

    thisPose3D.x = state_updated.pos(0);
    thisPose3D.y = state_updated.pos(1);
    thisPose3D.z = state_updated.pos(2);
    thisPose3D.intensity = cloudKeyPoses3D->size(); // store the index in intensity
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = state_updated.pos(0);
    thisPose6D.y = state_updated.pos(1);
    thisPose6D.z = state_updated.pos(2);
    thisPose6D.intensity = thisPose3D.intensity;

    Eigen::Vector3d eulerAngle = state_updated.rot.matrix().eulerAngles(2, 1, 0); // yaw pitch roll, rad
    thisPose6D.roll = eulerAngle(2);
    thisPose6D.pitch = eulerAngle(1);
    thisPose6D.yaw = eulerAngle(0);
    thisPose6D.time = lidar_end_time;
    thisPose6D.qw = state_updated.rot.w();
    thisPose6D.qx = state_updated.rot.x();
    thisPose6D.qy = state_updated.rot.y();
    thisPose6D.qz = state_updated.rot.z();
    cloudKeyPoses6D->push_back(thisPose6D);

    if (is_curr_degraded) {
        degraded_poses[idx_degraded_pose] = cloudKeyPoses3D->size() - 1;
        idx_degraded_pose = (idx_degraded_pose + 1) % degraded_poses.size();
        is_curr_degraded = false;
    }

    // TODO: modify covariance matrix
    // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P();
    // P_updated.setIdentity();
    // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
    // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
    // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
    // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
    // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
    // kf.change_P(P_updated);

    // pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    // pcl::copyPointCloud(*feats_undistort,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame);

    // cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    updatePath(thisPose6D);
}

void reconstructIKdTree() {
    if (reconstructKdTree && updateKdtreeCount > 0) {
        /*** if path is too large, the rvis will crash ***/
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // search for neighoring key frames in kdtree
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        mtx.lock();
        kdtreeGlobalMapPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMapPoses->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius,
                                           pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            subMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
        downSizeFilterSubMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
                                                 globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
        downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS);
        for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i) {
            // check if distance is too large
            if (pointDistance(subMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) >
                globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;
            // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
            // &cloudKeyPoses6D->points[thisKeyInd]);
            *subMapKeyFrames += *transformPointCloud(
                surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]); //  fast_lio only use  surfCloud
        }
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
                                                     globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*subMapKeyFramesDS);

        std::cout << "subMapKeyFramesDS sizes  =  " << subMapKeyFramesDS->points.size() << std::endl;

        ikdtree.reconstruct(subMapKeyFramesDS->points);
        updateKdtreeCount = 0;
        ROS_INFO("Reconstructed  ikdtree ");
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
        std::cout << "featsFromMapNum  =  " << featsFromMapNum << "\t"
                  << " kdtree_size_st   =  " << kdtree_size_st << std::endl;
    }
    updateKdtreeCount++;
}

/**
 * correct all poses in the path
 */
void correctPoses() {
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true) {
        // clear the path
        globalPath.poses.clear();
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i) {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
            gtsam::Quaternion q = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().toQuaternion();
            q.normalize();
            if (q.w() * cloudKeyPoses6D->points[i].qw < 0) {
                cloudKeyPoses6D->points[i].qw = -q.w();
                cloudKeyPoses6D->points[i].qx = -q.x();
                cloudKeyPoses6D->points[i].qy = -q.y();
                cloudKeyPoses6D->points[i].qz = -q.z();
            } else {
                cloudKeyPoses6D->points[i].qw = q.w();
                cloudKeyPoses6D->points[i].qx = q.x();
                cloudKeyPoses6D->points[i].qy = q.y();
                cloudKeyPoses6D->points[i].qz = q.z();
            }

            updatePath(cloudKeyPoses6D->points[i]);
        }
        // clear submap, reconstruct ikdtree submap
        reconstructIKdTree();
        ROS_INFO("ISMA2 Update");
        aLoopIsClosed = false;
    }
}
/**
 * Key factors that influence the loop detection
 * 1. minimum time difference, should be larger, like 30s, 1min;
 * 2. frequency of detection, not to high, around 1Hz
 */
bool detectLoopClosureDistance(int* latestID, int* closestID) {
    // current key frame
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // if detected already, return
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    std::vector<int> pointSearchIndLoop;     // candidate keyframe indices
    std::vector<float> pointSearchSqDisLoop; // candidate distance
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop,
                                        pointSearchSqDisLoop, 0);
    // choose the keyframe with longer time lag
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - lidar_end_time) > historyKeyframeSearchTimeDiff) {
            loopKeyPre = id;
            break;
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    ROS_INFO("Find loop clousre frame ");
    return true;
}

/**
 * get adjacent keyframes of loop closure frame and for a submap
 */
void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum) {
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    auto surfcloud_keyframes_size = surfCloudKeyFrames.size();
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;

        if (keyNear < 0 || keyNear >= surfcloud_keyframes_size)
            continue;

        // point cloud is in lidar frame, first transform to body frame, then world frame, cloudKeyPoses6D stores T_w_b
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // down sampling
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void performLoopClosure() {
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";

    if (cloudKeyPoses3D->points.empty() == true) {
        return;
    }

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre;
    for (int idx = 0; idx < degraded_poses.size(); ++idx) {
        if (degraded_poses[idx] == loopKeyCur)
            return;
    }

    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) {
        return;
    }

    for (int idx = 0; idx < degraded_poses.size(); ++idx) {
        if (degraded_poses[idx] == loopKeyPre)
            return;
    }
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>()); // current keyframe
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>()); // history keyframe submap
    {
        // get current point cloud in world frame
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
        // get submap around candidate keyframe in world frame
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
        // if feature points are too few, then return
        // if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        //     return;
        if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // scan-to-map
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // check the matching result
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    std::cout << "icp succeeded" << std::endl;

    if (pubIcpKeyFrames.getNumSubscribers() != 0) {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }

    // pose currection for current pose, according to the icp
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    Eigen::Quaterniond qq = EulerToQuat(roll, pitch, yaw);
    std::cout << "relative pos " << x << " " << y << " " << z << std::endl;
    std::cout << "relative att " << roll * 180 / 3.1415 << " " << pitch * 180 / 3.1415 << " " << yaw * 180 / 3.1415
              << std::endl;
    // current pose
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // current pose with correction from icp (world frame)
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    Eigen::Matrix3d rotm = tCorrect.rotation().cast<double>();
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3(rotm), gtsam::Point3(x, y, z));
    // history pose
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore(); //  loop_clousre  noise from icp
    noiseScore *= icp_noise_boost_ratio;
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    std::cout << "loopNoiseQueue  =  " << noiseScore << std::endl;

    // add loop data
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    loopIndexContainer[loopKeyCur] = loopKeyPre; // hash map
}

/**
 * loop closure thread, at user defined frequency
 */
void loopClosureThread() {
    if (loopClosureEnableFlag == false) {
        std::cout << "loopClosureEnableFlag   ==  false " << endl;
        return;
    }

    ros::Rate rate(loopClosureFrequency);
    while (ros::ok() && startFlag) {
        rate.sleep();
        performLoopClosure();
        visualizeLoopClosure();
    }
}

void SigHandle(int sig) {
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE* fp) {
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const* const pi, PointType* const po, state_ikfom& s) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const* const pi, PointType* const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    // world <-- imu <-- lidar
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T> void pointBodyToWorld(const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const* const pi, PointType* const po) //  lidar2world
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void test_RGBpointBodyToWorld(PointType const* const pi, PointType* const po, Eigen::Vector3d pos,
                              Eigen::Matrix3d rotation) //  lidar2world
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(rotation * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const* const pi, PointType* const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    for (int i = 0; i < points_history.size(); i++)
        _featsArray->push_back(points_history[i]);
}

// segment the point cloud according to fov
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world); // change boundary along X axis to world frame
    V3D pos_LiD = pos_lid;                               // lidar pose in global frame

    // initializae the local submap, centered as lidar pose in world frame
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2]; // distance to the edge of local map
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);

        // if too close to one edge, then move
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }

    if (!need_move)
        return;

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points; // new map points
    float mov_dist =
        max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints); // push the points need to be removed
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
               last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
        !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu; //????
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    // feature extraction or down sampling
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in) {
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // if time difference between lidar and imu is too large, then start synchronization
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp; // update imu time

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void gnss_cbk(const sensor_msgs::NavSatFixConstPtr& msg_in) {
    //  ROS_INFO("GNSS DATA IN ");
    double timestamp = msg_in->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_gnss) {
        ROS_WARN("gnss loop back, clear buffer");
        gnss_buffer.clear();
    }

    last_timestamp_gnss = timestamp;

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    gnss_data.time = msg_in->header.stamp.toSec();
    gnss_data.status = msg_in->status.status;
    gnss_data.service = msg_in->status.service;
    gnss_data.pose_cov[0] = msg_in->position_covariance[0];
    gnss_data.pose_cov[1] = msg_in->position_covariance[4];
    gnss_data.pose_cov[2] = msg_in->position_covariance[8];

    mtx_buffer.unlock();

    if (!gnss_inited) {
        gnss_data.InitOriginPosition(msg_in->latitude, msg_in->longitude, msg_in->altitude);
        gnss_inited = true;
    } else {
        gnss_data.UpdateXYZ(msg_in->latitude, msg_in->longitude, msg_in->altitude); //  WGS84 -> ENU ?? seem to be NED

        Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
        gnss_pose(0, 3) = gnss_data.local_N;
        gnss_pose(1, 3) = gnss_data.local_E;
        gnss_pose(2, 3) = -gnss_data.local_U;

        Eigen::Isometry3d gnss_to_lidar(Gnss_R_wrt_Lidar);
        gnss_to_lidar.pretranslate(Gnss_T_wrt_Lidar);
        gnss_pose = gnss_to_lidar * gnss_pose;

        nav_msgs::Odometry gnss_data_enu;
        // add new message to buffer:
        gnss_data_enu.header.stamp = ros::Time().fromSec(gnss_data.time);
        gnss_data_enu.pose.pose.position.x = gnss_pose(0, 3);
        gnss_data_enu.pose.pose.position.y = gnss_pose(1, 3);
        gnss_data_enu.pose.pose.position.z = gnss_pose(2, 3);

        gnss_data_enu.pose.pose.orientation.x = geoQuat.x;
        gnss_data_enu.pose.pose.orientation.y = geoQuat.y;
        gnss_data_enu.pose.pose.orientation.z = geoQuat.z;
        gnss_data_enu.pose.pose.orientation.w = geoQuat.w;

        gnss_data_enu.pose.covariance[0] = gnss_data.pose_cov[0];
        gnss_data_enu.pose.covariance[7] = gnss_data.pose_cov[1];
        gnss_data_enu.pose.covariance[14] = gnss_data.pose_cov[2];

        gnss_buffer.push_back(gnss_data_enu);

        // visial gnss path in rviz:
        msg_gnss_pose.header.frame_id = "camera_init";
        msg_gnss_pose.header.stamp = ros::Time().fromSec(gnss_data.time);

        msg_gnss_pose.pose.position.x = gnss_pose(0, 3);
        msg_gnss_pose.pose.position.y = gnss_pose(1, 3);
        msg_gnss_pose.pose.position.z = gnss_pose(2, 3);

        gps_path.poses.push_back(msg_gnss_pose);

        //  save_gnss path
        PointTypePose thisPose6D;
        thisPose6D.x = msg_gnss_pose.pose.position.x;
        thisPose6D.y = msg_gnss_pose.pose.position.y;
        thisPose6D.z = msg_gnss_pose.pose.position.z;
        thisPose6D.intensity = 0;
        thisPose6D.roll = 0;
        thisPose6D.pitch = 0;
        thisPose6D.yaw = 0;
        thisPose6D.time = lidar_end_time;
        gnss_cloudKeyPoses6D->push_back(thisPose6D);
    }
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool sync_packages(MeasureGroup& meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();

        if (meas.lidar->points.size() <= 1) {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; // record lidar end time
            ROS_WARN("Too few input point cloud!\n");
        } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        } else {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            // dynamically update the mean scan time of lidar
            lidar_mean_scantime +=
                (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental() {
    PointVector PointToAdd;            // points to be added to ikdtree
    PointVector PointNoNeedDownsample; // pointcloud that does not need down sampling when adding to ikdtree
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    // check the distance to box center
    for (int i = 0; i < feats_down_size; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector& points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            float dist =
                calc_dist(feats_down_world->points[i], mid_point); // distance between current point and box center

            // check the manhattan distance betwee nearest point and center point
            // if distance along 3 axis are larger than half of filter size, then no down sampling
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }

            // check current neighboring points' distance to center
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                // if neighboring point's distance is smaller than distance between current point and center
                // then do not add neighboring point
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher& pubLaserCloudFull) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher& pubLaserCloudFull_body) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher& pubLaserCloudEffect) {
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
        RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher& pubLaserCloudMap) {
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T> void set_posestamp(T& out) {
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = state_point.rot.x();
    out.pose.orientation.y = state_point.rot.y();
    out.pose.orientation.z = state_point.rot.z();
    out.pose.orientation.w = state_point.rot.w();
}

void publish_loop_constrain(const ros::Publisher& pubLoopConstrain) {
    if (loopIndexQueue.empty())
        return;

    // loop closure queue
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
        // loop nodes
        int indexFrom = loopIndexQueue[i].first; // cur
        int indexTo = loopIndexQueue[i].second;  // pre
        // pose transform
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        loopConstrain.header.frame_id = "loop_constrain";
        loopConstrain.child_frame_id = std::to_string(cloudKeyPoses6D->points[indexTo].time);
        loopConstrain.header.stamp =
            ros::Time().fromSec(cloudKeyPoses6D->points[indexFrom].time); // ros::Time().fromSec(lidar_end_time);
        loopConstrain.pose.pose.position.x = poseBetween.x();
        loopConstrain.pose.pose.position.y = poseBetween.y();
        loopConstrain.pose.pose.position.z = poseBetween.z();
        gtsam::Rot3 R_loop = poseBetween.rotation();
        gtsam::Quaternion q_loop = R_loop.toQuaternion();
        loopConstrain.pose.pose.orientation.w = q_loop.w();
        loopConstrain.pose.pose.orientation.x = q_loop.x();
        loopConstrain.pose.pose.orientation.y = q_loop.y();
        loopConstrain.pose.pose.orientation.z = q_loop.z();
        pubLoopConstrain.publish(loopConstrain);
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

void publish_odometry(const ros::Publisher& pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    odomAftMapped.twist.twist.linear.x = state_point.vel(0);
    odomAftMapped.twist.twist.linear.y = state_point.vel(1);
    odomAftMapped.twist.twist.linear.z = state_point.vel(2);
    if (is_benefit_from_lc) {
        odomAftMapped.child_frame_id = "loop";
        is_benefit_from_lc = false;
    }
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }
    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void publish_path(const ros::Publisher pubPath) {
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);

        //  save  unoptimized pose
        V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
        PointTypePose thisPose6D;
        thisPose6D.x = msg_body_pose.pose.position.x;
        thisPose6D.y = msg_body_pose.pose.position.y;
        thisPose6D.z = msg_body_pose.pose.position.z;
        thisPose6D.roll = rot_ang(0);
        thisPose6D.pitch = rot_ang(1);
        thisPose6D.yaw = rot_ang(2);
        fastlio_unoptimized_cloudKeyPoses6D->push_back(thisPose6D);
    }
}

void publish_path_update(const ros::Publisher pubPath) {
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    if (pubPath.getNumSubscribers() != 0) {
        /*** if path is too large, the rvis will crash ***/
        static int kkk = 0;
        kkk++;
        if (kkk % 10 == 0) {
            // path.poses.push_back(globalPath);
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
}

void publish_gnss_path(const ros::Publisher pubPath) {
    gps_path.header.stamp = ros::Time().fromSec(lidar_end_time);
    gps_path.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        pubPath.publish(gps_path);
    }
}

struct pose {
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
};

bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path, std::ios::out);
    if (!ofs) {
        std::cout << "open csv file error " << std::endl;
        return false;
    }
    return true;
}

/* write2txt   format  KITTI*/
void WriteText(std::ofstream& ofs, pose data) {
    ofs << std::fixed << data.R(0, 0) << " " << data.R(0, 1) << " " << data.R(0, 2) << " " << data.t[0] << " "
        << data.R(1, 0) << " " << data.R(1, 1) << " " << data.R(1, 2) << " " << data.t[1] << " " << data.R(2, 0) << " "
        << data.R(2, 1) << " " << data.R(2, 2) << " " << data.t[2] << std::endl;
}

bool savePoseService(fast_lio_sam::save_poseRequest& req, fast_lio_sam::save_poseResponse& res) {
    pose pose_gnss;
    pose pose_optimized;
    pose pose_without_optimized;

    std::ofstream file_pose_gnss;
    std::ofstream file_pose_optimized;
    std::ofstream file_pose_without_optimized;

    string savePoseDirectory;
    cout << "****************************************************" << endl;
    cout << "Saving poses to pose files ..." << endl;
    if (req.destination.empty())
        savePoseDirectory = std::getenv("HOME") + savePCDDirectory;
    else
        savePoseDirectory = std::getenv("HOME") + req.destination;
    cout << "Save destination: " << savePoseDirectory << endl;

    // create file
    CreateFile(file_pose_gnss, savePoseDirectory + "/gnss_pose.txt");
    CreateFile(file_pose_optimized, savePoseDirectory + "/optimized_pose.txt");
    CreateFile(file_pose_without_optimized, savePoseDirectory + "/without_optimized_pose.txt");

    //  save optimize data
    for (int i = 0; i < cloudKeyPoses6D->size(); i++) {
        pose_optimized.t =
            Eigen::Vector3d(cloudKeyPoses6D->points[i].x, cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z);
        pose_optimized.R = Exp(double(cloudKeyPoses6D->points[i].roll), double(cloudKeyPoses6D->points[i].pitch),
                               double(cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_optimized, pose_optimized);
    }
    cout << "Sucess global optimized  poses to pose files ..." << endl;

    for (int i = 0; i < fastlio_unoptimized_cloudKeyPoses6D->size(); i++) {
        pose_without_optimized.t = Eigen::Vector3d(fastlio_unoptimized_cloudKeyPoses6D->points[i].x,
                                                   fastlio_unoptimized_cloudKeyPoses6D->points[i].y,
                                                   fastlio_unoptimized_cloudKeyPoses6D->points[i].z);
        pose_without_optimized.R = Exp(double(fastlio_unoptimized_cloudKeyPoses6D->points[i].roll),
                                       double(fastlio_unoptimized_cloudKeyPoses6D->points[i].pitch),
                                       double(fastlio_unoptimized_cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_without_optimized, pose_without_optimized);
    }
    cout << "Sucess unoptimized  poses to pose files ..." << endl;

    for (int i = 0; i < gnss_cloudKeyPoses6D->size(); i++) {
        pose_gnss.t = Eigen::Vector3d(gnss_cloudKeyPoses6D->points[i].x, gnss_cloudKeyPoses6D->points[i].y,
                                      gnss_cloudKeyPoses6D->points[i].z);
        pose_gnss.R = Exp(double(gnss_cloudKeyPoses6D->points[i].roll), double(gnss_cloudKeyPoses6D->points[i].pitch),
                          double(gnss_cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_gnss, pose_gnss);
    }
    cout << "Sucess gnss  poses to pose files ..." << endl;

    file_pose_gnss.close();
    file_pose_optimized.close();
    file_pose_without_optimized.close();
    return true;
}

bool saveMapService(fast_lio_sam::save_mapRequest& req, fast_lio_sam::save_mapResponse& res) {
    string saveMapDirectory;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;
    if (req.destination.empty())
        saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
    else
        saveMapDirectory = std::getenv("HOME") + req.destination;
    cout << "Save destination: " << saveMapDirectory << endl;
    // int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
    // unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
    // save keyframe poses
    pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
    pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);

    // extract keyframes' corner/plane feature points
    // pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

    // keyframe point clouds are in lidar frame, cloudKeyPoses6D are in body frame
    // T_world_lidar = T_world_body * T_body_lidar , T_body_lidar is extrinsic
    for (int i = 0; i < (int)cloudKeyPoses6D->size(); i++) {
        // *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }

    if (req.resolution != 0) {
        cout << "\n\nSave resolution: " << req.resolution << endl;

        // downSizeFilterCorner.setInputCloud(globalCornerCloud);
        // downSizeFilterCorner.setLeafSize(req.resolution, req.resolution, req.resolution);
        // downSizeFilterCorner.filter(*globalCornerCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
    } else {
        //   downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);           // dense
        // pointcloud map
    }

    // *globalMapCloud += *globalCornerCloud;
    *globalMapCloud += *globalSurfCloud;
    pcl::io::savePCDFileBinary(saveMapDirectory + "/filterGlobalMap.pcd",
                               *globalSurfCloudDS); // down sampled point cloud map
    int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud); // dense point cloud map
    res.success = ret == 0;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;

    // visial optimize global map on viz
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    publishCloud(&pubOptimizedGlobalMap, globalSurfCloudDS, timeLaserInfoStamp, odometryFrame);

    return true;
}

void saveMap() {
    fast_lio_sam::save_mapRequest req;
    fast_lio_sam::save_mapResponse res;
    if (!saveMapService(req, res)) {
        cout << "Fail to save map" << endl;
    }
}

void publishGlobalMap() {
    /*** if path is too large, the rvis will crash ***/
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    if (pubLaserCloudSurround.getNumSubscribers() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
    ;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // search for neighboring keyframes of current keyframe from kdtree
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap,
                                  pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;

    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
                                                globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
        // check if distance is too large
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
        // &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(
            surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]); //  fast_lio only use  surfCloud
    }

    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
                                                 globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
}

// construct H matrix
void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) // check if the neighboring points of current point can form a plane
    {
        PointType& point_body = feats_down_body->points[i];   // lidar frame
        PointType& point_world = feats_down_world->points[i]; // world frame

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto& points_near = Nearest_Points[i];

        if (ekfom_data.converge) {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS
                                         ? false
                                         : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4) pabcd; // plane parameter,  a b c d
        point_selected_surf[i] = false;
        // fit the local plane
        if (esti_plane(pabcd, points_near, 0.1f)) {
            // plane distance
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s =
                1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); // 1 - 0.9 * (distance to plane / distance to lidar origin)

            if (s > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2; // record the residual into intensity
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num; // mean value of the resduals
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType& laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType& norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z); // local norm vector, world frame

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec); // rotate the local norm vector to body frame, corr_normal_I
        V3D A(point_crossmat * C);           // d_residual / d_angle, P(IMU)^ [R(imu <-- w) * normal_w]
        if (extrinsic_est_en) {
            // B = lidar_p^ R(L <-- I) * corr_normal_I
            // B = lidar_p^ R(L <-- I) * R(I <-- W) * normal_W
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B),
                VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv) {
    // allocateMemory();
    for (int i = 0; i < 6; ++i) {
        transformTobeMapped[i] = 0;
    }

    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<double>("mapping/fov_degree", fov_deg, 180);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<double>("filter_x_lower", filter_x_lower, 0);
    nh.param<double>("filter_x_upper", filter_x_upper, 0);
    nh.param<double>("filter_y_lower", filter_y_lower, 0);
    nh.param<double>("filter_y_upper", filter_y_upper, 0);
    nh.param<double>("filter_z_lower", filter_z_lower, 0);
    nh.param<double>("filter_z_upper", filter_z_upper, 0);
    nh.param<double>("HTH_eigen_val_thres", HTH_eigen_val_thres, 30);
    nh.param<double>("icp_noise_boost_ratio", icp_noise_boost_ratio, 1);
    nh.param<bool>("is_front_backend_tightly_coupled", is_front_backend_tightly_coupled, false);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    cout << "p_pre->lidar_type " << p_pre->lidar_type << endl;

    nh.param<float>("odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    nh.param<float>("mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    nh.param<float>("mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    nh.param<float>("z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("rotation_tollerance", rotation_tollerance, FLT_MAX);

    nh.param<int>("numberOfCores", numberOfCores, 2);
    nh.param<double>("mappingProcessInterval", mappingProcessInterval, 0.15);

    // save keyframes
    nh.param<float>("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 20.0);
    nh.param<float>("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

    // loop clousre
    nh.param<bool>("loopClosureEnableFlag", loopClosureEnableFlag, false);
    nh.param<float>("loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<int>("surroundingKeyframeSize", surroundingKeyframeSize, 50);
    nh.param<float>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<float>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

    // gnss
    nh.param<string>("common/gnss_topic", gnss_topic, "/gps/fix");
    nh.param<vector<double>>("mapping/extrinR_Gnss2Lidar", extrinR_Gnss2Lidar, vector<double>());
    nh.param<vector<double>>("mapping/extrinT_Gnss2Lidar", extrinT_Gnss2Lidar, vector<double>());
    nh.param<bool>("useImuHeadingInitialization", useImuHeadingInitialization, false);
    nh.param<bool>("useGpsElevation", useGpsElevation, false);
    nh.param<float>("gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("poseCovThreshold", poseCovThreshold, 25.0);

    // Visualization
    nh.param<float>("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    // visual ikdtree map
    nh.param<bool>("visulize_IkdtreeMap", visulize_IkdtreeMap, false);

    // reconstruct ikdtree
    nh.param<bool>("reconstructKdTree", reconstructKdTree, false);

    // savMap
    nh.param<bool>("savePCD", savePCD, false);
    nh.param<std::string>("savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    // downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(
        surroundingKeyframeDensity, surroundingKeyframeDensity,
        surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    // ISAM2 parameters
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
                           aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    Gnss_T_wrt_Lidar << VEC_FROM_ARRAY(extrinT_Gnss2Lidar);
    Gnss_R_wrt_Lidar << MAT_FROM_ARRAY(extrinR_Gnss2Lidar);

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);

    // init kalman filter, defining functions to calculate F, H, ...
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE* fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk)
                                                        : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000); //  dense pcl world
    ros::Publisher pubLaserCloudFull_body =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000); //  dense pcl body
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000); //  not used
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);         //  not used
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubLoopConstrain = nh.advertise<nav_msgs::Odometry>("/Loop", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 1e00000);

    ros::Publisher pubPathUpdate =
        nh.advertise<nav_msgs::Path>("fast_lio_sam/path_update", 100000); // path after update by isam
    pubGnssPath = nh.advertise<nav_msgs::Path>("/gnss_path", 100000);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/keyframe_submap", 1);
    pubOptimizedGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/map_global_optimized", 1);

    // loop clousre
    pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_history_cloud",
                                                                 1); // submap to match current keyframe
    pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_corrected_cloud",
                                                             1); // current pcl after loop closure
    pubLoopConstraintEdge =
        nh.advertise<visualization_msgs::MarkerArray>("/fast_lio_sam/mapping/loop_closure_constraints", 1);

    ros::Subscriber sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_cbk);

    srvSaveMap = nh.advertiseService("/save_map", &saveMapService);
    srvSavePose = nh.advertiseService("/save_pose", &savePoseService);

    // loop closure
    std::thread loopthread(&loopClosureThread);

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status) {
        if (flg_exit)
            break;
        ros::spinOnce();

        if (sync_packages(Measures)) // Measurement stores current lidar frame and corresponding imu data
        {
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf,
                           feats_undistort); // forward propagation, deskew pcl, feats_undistort is in lidar frame
            state_point = kf.get_x();        // pose after imu prediction
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL)) {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment(); // redefine box, remove points too far away

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();

            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr) {
                if (feats_down_size > 5) {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++) {
                        pointBodyToWorld(&(feats_down_body->points[i]),
                                         &(feats_down_world->points[i])); // from lidar to world
                    }
                    // initialize kdtree
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num:
            // "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5) {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            // lidar --> imu
            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " "
                     << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                     << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
                     << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav
                     << endl;

            if (visulize_IkdtreeMap) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
                publish_map(pubLaserCloudMap);
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            /*** iterated state estimation ***/

            t2 = omp_get_wtime();
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            // measurement update of kalman filter
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            double t_update_end = omp_get_wtime();

            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // current pose in world frame
            geoQuat.x = state_point.rot.x();
            geoQuat.y = state_point.rot.y();
            geoQuat.z = state_point.rot.z();
            geoQuat.w = state_point.rot.w();

            getCurPose(state_point); // update transformTobeMapped

            /*** iterated state estimation ***/
            // 1.calculate relative pose w.r.t. previous keyframe, if large enough, set as keyframe
            // 2.add relative pose factor, GNSS factor, loop closure factor
            // 3.optimization
            // 4.update pose, and covariance (todo)
            // 5.update cloudKeyPoses3D, cloudKeyPoses6D, transformTobeMapped
            saveKeyFramesAndFactor();

            // update the path, reconstruct kdtree
            correctPoses();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
            // publish_loop_constrain(pubLoopConstrain);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            /******* Publish points *******/
            if (path_en) {
                publish_path(pubPath);
                publish_gnss_path(pubGnssPath);
                publish_path_update(pubPathUpdate); // path after optimization
                static int jjj = 0;
                jjj++;
                if (jjj % 100 == 0) {
                    publishGlobalMap();
                }
            }
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body);

            // if(savePCD)  saveMap();

            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log) {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp =
                    aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve =
                    aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                printf("[ mapping ]: time: %.4f IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  "
                       "ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f isam: %.6f\n",
                       lidar_end_time, t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
                       aver_time_icp, aver_time_const_H_time, t_isam_end - t_isam_start);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose()
                         << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                         << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
                         << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav
                         << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log) {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE* fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, delete "
                     "time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++) {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]),
                    s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]),
                    int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    startFlag = false;
    loopthread.join();

    return 0;
}
