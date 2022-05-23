#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

// #include "cloud_msgs/cloud_info.h"
#include "rdc_slam/cloud_info.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//#include <DistributedMapper.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <time.h>

#include <Eigen/Core>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;
typedef pcl::PointXYZ  PclPoint;

const bool send_map = true;//是否发送地图

// delight
const int I_BIN = 64;//intensity维数 ----填
const int delight_accum_frames = 1;
const double delight_similarity_threshold = 600.0;//I_BIN=64时
const int inner_sphere_r1 = 30;
const int outer_sphere_r2 = 100;

// segment_eigen
extern const int knn_feature_dim = 7;
int n_nearest_neighbours = 3;
double feature_distance_threshold = 5;//0.2
double gc_resolution = 0.25;//inlier threshold 0.15
int gc_min_cluster_size = 5;//7
int segmentation_mode = 2;//mode=1是range_image based分割，mode=2 Euclidean

// dgo
extern const string ROBOT_NAMES = string("abcdefghijklmnopqrstyvwxyz");
extern const double rotationEstimateChangeThreshold = 1e-2;//5;
extern const double poseEstimateChangeThreshold = 1e-2;//5;
extern const int distMappers_max_iter=500;

const double SIM_COMM_RANGE = 1000.0;
const int SEPNODE_STEP = 2;

// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;
extern const string imuTopic = "/imu/data";


extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472;
extern const int segmentValidPointNum = 15;
extern const int segmentValidLineNum = 5;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 50.0;
extern const int   surroundingKeyframeSearchNum = 50;

extern const float historyKeyframeSearchRadius = 5.0;
extern const int   historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.3;

extern const float globalMapVisualizationSearchRadius = 500.0;


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
