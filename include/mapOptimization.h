#ifndef _MAP_OPTIMIZATION_H_
#define _MAP_OPTIMIZATION_H_

#include "utility.h"
#include "msgFactory.h"
#include "delight.h"
#include "segmentFeatures.h"

#include "rdc_slam/OutputMap.h"
#include "rdc_slam/OutputTraj.h"
#include "rdc_slam/OutputSepNodePcd.h"

#include <distributed_mapper/DistributedMapper.h>
#include <gtsam/slam/dataset.h>

using namespace gtsam;
using namespace distributed_mapper;


struct StampedRobotMessage{
  int refVertex; 
  RobotMessage* msg;
};

class mapOptimization{

private:

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    // ISAM2 *isam;
    Values isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    noiseModel::Diagonal::shared_ptr constraintNoise;
    noiseModel::Diagonal::shared_ptr outerEdgeNoise;
    ros::NodeHandle nh;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubKeyPoses;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;

    ros::Publisher pubintermap;

    ros::Subscriber subLaserCloudCornerLast;
    ros::Subscriber subLaserCloudSurfLast;
    ros::Subscriber subOutlierCloudLast;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subImu;

    ros::Subscriber subLaserCloudRawLast;
    ros::Subscriber subLaserCloudSegmentLast;

    ros::ServiceServer output_traj_server;
    ros::ServiceServer output_map_server;
    ros::ServiceServer outputSepNodePcd_server;

    nav_msgs::Odometry odomAftMapped;
    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;

    vector<pcl::PointCloud<PointType>::Ptr> segmentCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> delightCloudKeyFrames;

    deque<pair<int,pcl::PointCloud<PointType>::Ptr> > recentRawCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
    int latestFrameID;
    int Framecnt;

    vector<int> surroundingExistingKeyPosesID;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;
    
    PointType previousRobotPosPoint;
    PointType currentRobotPosPoint;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;

    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast;
    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLastDS;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS;

    pcl::PointCloud<PointType>::Ptr laserCloudRawLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSegmentLast;

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    
    pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
    pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

    pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

    pcl::PointCloud<PointType>::Ptr submapKeyFrames;
    pcl::PointCloud<PointType>::Ptr submapKeyFramesDS;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    std::vector<int> pointSearchInd2;
    std::vector<float> pointSearchSqDis2;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterOutlier;
    pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
    pcl::VoxelGrid<PointType> downSizeFilterSubmapKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFramesoutput;

    double timeLaserCloudCornerLast;
    double timeLaserCloudSurfLast;
    double timeLaserOdometry;
    double timeLaserCloudOutlierLast;
    double timeLastGloalMapPublish;
    double timeLaserCloudRawLast;
    double timeLaserCloudSegmentLast;

    bool newLaserCloudCornerLast;
    bool newLaserCloudSurfLast;
    bool newLaserOdometry;
    bool newLaserCloudOutlierLast;
    bool newLaserCloudRawLast;
    bool newLaserCloudSegmentLast;

    float transformLast[6];
    float transformSum[6];
    float transformIncre[6];
    float transformTobeMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];

    int imuPointerFront;
    int imuPointerLast;

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];

    std::mutex mtx;
    std::mutex graph_mtx;
    std::mutex recentRawCloudKeyFrames_mtx;
    std::mutex copy_mtx;

    double timeLastProcessing;

    PointType pointOri, pointSel, pointProj, coeff;

    cv::Mat matA0;
    cv::Mat matB0;
    cv::Mat matX0;

    cv::Mat matA1;
    cv::Mat matD1;
    cv::Mat matV1;

    bool isDegenerate;
    cv::Mat matP;

    cv::Mat matA02;
    cv::Mat matB02;
    cv::Mat matX02;

    cv::Mat matA12;
    cv::Mat matD12;
    cv::Mat matV12;

    bool isDegenerate2;
    cv::Mat matP2;

    int laserCloudCornerFromMapDSNum;
    int laserCloudSurfFromMapDSNum;
    int laserCloudCornerLastDSNum;
    int laserCloudSurfLastDSNum;
    int laserCloudOutlierLastDSNum;
    int laserCloudSurfTotalLastDSNum;

    bool potentialLoopFlag;
    double timeSaveFirstCurrentScanForLoopClosure;
    int closestHistoryFrameID;
    int latestFrameIDLoopCloure;

    bool aLoopIsClosed;
    bool isRefresh;

    float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
    float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

    MessageFactory *_factoryHandler;

    int idRobot_;
    int nRobots_;
    int baseIdx_;

    std::stringstream rootnsFrameId;
    std::string rootnsFrameIdStr;

    boost::mutex graphMutex;

    std::vector<int> lastOutClosure;
    std::vector<int> lastOutClosuremap;

    vector<pcl::PointCloud<PointType>::Ptr> interMap;
    vector<int> inclosures[5];

    //==========scan_match================
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap2;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap2;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap2;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap2;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS2;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS2;

    int laserCloudCornerFromMapDSNum2;
    int laserCloudSurfFromMapDSNum2;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses2;

    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS2;
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses2; 

    pcl::PointCloud<PointType>::Ptr laserCloudOri2;
    pcl::PointCloud<PointType>::Ptr coeffSel2;

    PointType pointOri2, pointSel2, coeff2;

    float transformTobeMapped2[6];
    //==============scan_match===========

    bool flagMapSend;

    std::vector<int> lastSepNode_;

    int featureCnt;
    Eigen::MatrixXi featureMatrix;

    std::vector<std::pair<int,int> > candidateIdPairs;

    double timePR;
    double timeRP;
    double timeDGO;

    // std::deque<pair<int,GraphMessage*>> outGraphMsg;
    std::deque<pair<int, MapMessage*>> outMapMsg;
    std::deque<DistGraphMessage*> outDistGraphMsg;
    std::deque<RPMessage*> outRPMsg;

    char robotName;

    boost::shared_ptr<DistributedMapper> distMapperHandler;
    int iterCnt;
    bool firstStage;
    bool firstDGO;

    bool sendPRMSG;
    bool sendRPMSG;

    int lastSavedIndex;

    int sendMapClock;

public:
    mapOptimization(ros::NodeHandle nh_, int idRobot_ = 0, int nRobots_=0, int baseIdx_ = 10000);

    void allocateMemory();

    void transformAssociateToMap();
    void transformUpdate();
    void updatePointAssociateToMapSinCos();
    void pointAssociateToMap(PointType const * const pi, PointType * const po);
    void updateTransformPointCloudSinCos(PointTypePose *tIn);
    
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
    
    void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);
    void laserCloudRawLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laserCloudSegmentLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);

    void publishTF();
    void publishKeyPosesAndFrames();
    void visualizeGlobalMapThread();
    void publishGlobalMap();
    
    void loopClosureThread();
    bool detectLoopClosure();
    void performLoopClosure();

    void adjustCoordinate(PointTypePose* transformIn, PointTypePose* transformOut);
    void featureMatrixThread();
    void computeFeatureMatrix();

    Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
    Eigen::Affine3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint);
    void extractSurroundingKeyFrames();

    void downsampleCurrentScan();
    
    void cornerOptimization(int iterCount);
    void surfOptimization(int iterCount);
    bool LMOptimization(int iterCount);

    void scan2MapOptimization();

    void saveKeyFramesAndFactor();
    void correctPoses();

    void clearCloud();
    
    void run();

    int idRobot();
    int lastVertex();
    int lastSepNode(int toRobotId){
      return lastSepNode_[toRobotId];
    };

    double getTimePR(){
      return timePR;
    };
    double getTimeRP(){
      return timeRP;
    };
    double getTimeDGO(){
      return timeDGO;
    };

    RobotMessage* createMsgfromCharArray(const char* buffer, size_t size);
  
    ComboMessage* constructComboMessage();
    RPMessage* constructRPMessage(std::vector<std::pair<int,int> > candidateIdPairs);
    DistGraphMessage* constructDistGraphMessage(bool fin = false, NonlinearFactor::shared_ptr factor = NULL, int to_robot_id = -1);
    MapMessage* constructMapMessage(pcl::PointCloud<PointType>::Ptr pointcloud, int inclosure, int outclosure);

    void addInterRobotData(StampedRobotMessage smsg);
    // void addInterRobotData(ComboMessage* cmsg, int refVertex);
    void addInterRobotData(ComboMessage* cmsg);
    void addInterRobotData(RPMessage* gmsg);
    void addInterRobotData(DistGraphMessage* dgmsg);
    void addInterRobotData(MapMessage* mmsg);

    bool outputTraj_service(rdc_slam::OutputTrajRequest& req, rdc_slam::OutputTrajResponse& res);
    bool outputMap_service(rdc_slam::OutputMapRequest& req, rdc_slam::OutputMapResponse& res);
    bool outputSepNodePcd_service(rdc_slam::OutputSepNodePcdRequest& req, rdc_slam::OutputSepNodePcdResponse& res);
};
#endif


