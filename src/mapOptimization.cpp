#include "mapOptimization.h"

mapOptimization::mapOptimization(ros::NodeHandle nh_, int idRobot_, int nRobots_, int baseIdx_)
{
    rootnsFrameId << "/robot" << idRobot;
    rootnsFrameId >> rootnsFrameIdStr;

    factoryHandler = new MessageFactory();

    factoryHandler->registerMessageType<ComboMessage>();
    factoryHandler->registerMessageType<RPMessage>();
    factoryHandler->registerMessageType<DistGraphMessage>();
    factoryHandler->registerMessageType<MapMessage>();

    lastSepNode.resize(nRobots_);
    for(int i=0; i<nRobots_; i++){
        lastSepNode[i] = -100;
    }

    lastOutClosure.resize(nRobots_);
    lastOutClosuremap.resize(nRobots_);
    for(int i=0;i<nRobots_;i++){
        lastOutClosure[i] = 0;
        lastOutClosuremap[i] = -1;
    }

    idRobot = idRobot_;
    baseIdx = baseIdx_;
    nRobots = nRobots_;
    nh = nh_;

    robotName = ROBOT_NAMES[idRobot];
    // ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    // isam = new ISAM2(parameters);
    ROS_INFO("_distMapperHandler INI");
    distMapperHandler = boost::make_shared<DistributedMapper>(_robotName, false, true);
    distMapperHandler->updateInitialized(false);
    distMapperHandler->setUseBetweenNoiseFlag(true);
    iterCnt = 0;
    // if(idRobot == 0){
    //     distMapperHandler->updateInitialized(true);
    // }
    distMapperHandler->setVerbosity(_distMapperHandler->ERROR);
    ROS_INFO("_distMapperHandler INI_END");


    pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("key_pose_origin", 2);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_surround", 2);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("aft_mapped_to_init", 5);

    pubintermap = nh.advertise<sensor_msgs::PointCloud2>("inter_map", 2);

    subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);
    subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);
    subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>("outlier_cloud_last", 2, &mapOptimization::laserCloudOutlierLastHandler, this);
    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("laser_odom_to_init", 5, &mapOptimization::laserOdometryHandler, this);
    subImu = nh.subscribe<sensor_msgs::Imu> (imuTopic, 50, &mapOptimization::imuHandler, this);

    // subLaserCloudRawLast = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, &mapOptimization::laserCloudRawLastHandler, this);//xyt 5.27
    subLaserCloudRawLast = nh.subscribe<sensor_msgs::PointCloud2>("ground_removed_with_intensity", 10, &mapOptimization::laserCloudRawLastHandler, this);//12.03    
    subLaserCloudSegmentLast = nh.subscribe<sensor_msgs::PointCloud2>("segmented_cloud_pure", 10, &mapOptimization::laserCloudSegmentLastHandler, this);//xyt 5.27    

    pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("history_cloud", 2);
    pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("corrected_cloud", 2);
    pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("recent_cloud", 2);

    output_traj_server = nh.advertiseService("/cslam/outputTraj", &mapOptimization::outputTraj_service, this);
    output_map_server = nh.advertiseService("/cslam/outputMap", &mapOptimization::outputMap_service, this);
    outputSepNodePcd_server = nh.advertiseService("/cslam/outputSepPcd", &mapOptimization::outputSepNodePcd_service, this);

    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

    downSizeFilterHistoryKeyFrames.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0);

    downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0);
    downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4);

    downSizeFilterGlobalMapKeyFramesoutput.setLeafSize(0.5, 0.5, 0.5);

    downSizeFilterSubmapKeyPoses.setLeafSize(0.1, 0.1, 0.1);//xyt 5.28

    odomAftMapped.header.frame_id = rootnsFrameIdStr+"/camera_init";
    odomAftMapped.child_frame_id = rootnsFrameIdStr+"/aft_mapped";

    aftMappedTrans.frame_id_ = rootnsFrameIdStr+"/camera_init";
    aftMappedTrans.child_frame_id_ = rootnsFrameIdStr+"/aft_mapped";

    timePR = 0.0;
    timeRP = 0.0;
    timeDGO = 0.0;

    firstStage = true;
    sendPRMSG = true;
    sendRPMSG = true;

    last_saved_index = -1;
    sendMapClock = 0;

    allocateMemory();
}

void mapOptimization::allocateMemory(){

    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurroundingKeyPoses2.reset(new pcl::KdTreeFLANN<PointType>());//=============
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
    surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());  

    surroundingKeyPoses2.reset(new pcl::PointCloud<PointType>());//============
    surroundingKeyPosesDS2.reset(new pcl::PointCloud<PointType>());//=================      

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());
    laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>());
    laserCloudOutlierLastDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfTotalLastDS.reset(new pcl::PointCloud<PointType>());

    laserCloudRawLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSegmentLast.reset(new pcl::PointCloud<PointType>());

    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    laserCloudOri2.reset(new pcl::PointCloud<PointType>());
    coeffSel2.reset(new pcl::PointCloud<PointType>());

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    laserCloudCornerFromMap2.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap2.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS2.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS2.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeCornerFromMap2.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap2.reset(new pcl::KdTreeFLANN<PointType>());
    
    nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
    nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
    globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
    globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

    submapKeyFrames.reset(new pcl::PointCloud<PointType>());
    submapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

    timeLaserCloudCornerLast = 0;
    timeLaserCloudSurfLast = 0;
    timeLaserOdometry = 0;
    timeLaserCloudOutlierLast = 0;
    timeLastGloalMapPublish = 0;
    timeLaserCloudRawLast = 0;
    timeLaserCloudSegmentLast = 0;

    timeLastProcessing = -1;

    newLaserCloudCornerLast = false;
    newLaserCloudSurfLast = false;

    newLaserOdometry = false;
    newLaserCloudOutlierLast = false;

    newLaserCloudRawLast = false;
    newLaserCloudSegmentLast = false;

    for (int i = 0; i < 6; ++i){
        transformLast[i] = 0;
        transformSum[i] = 0;
        transformIncre[i] = 0;
        transformTobeMapped[i] = 0;
        transformBefMapped[i] = 0;
        transformAftMapped[i] = 0;
        transformTobeMapped2[i] = 0;

    imuPointerFront = 0;
    imuPointerLast = -1;

    for (int i = 0; i < imuQueLength; ++i){
        imuTime[i] = 0;
        imuRoll[i] = 0;
        imuPitch[i] = 0;
    }

    gtsam::Vector Vector6(6);
    Vector6 << 1, 1, 1, 1e-3, 1e-3, 1;   
    priorNoise = noiseModel::Diagonal::Variances(Vector6);
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);
    Vector6 << 1, 1, 1, 1, 1, 1;
    outerEdgeNoise = noiseModel::Diagonal::Variances(Vector6);

    matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
    matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
    matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));

    matA1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
    matD1 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
    matV1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

    isDegenerate = false;
    matP = cv::Mat (6, 6, CV_32F, cv::Scalar::all(0));

    matA02 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
    matB02 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
    matX02 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));

    matA12 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
    matD12 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
    matV12 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

    isDegenerate2 = false;
    matP2 = cv::Mat (6, 6, CV_32F, cv::Scalar::all(0));

    laserCloudCornerFromMapDSNum = 0;
    laserCloudSurfFromMapDSNum = 0;
    laserCloudCornerLastDSNum = 0;
    laserCloudSurfLastDSNum = 0;
    laserCloudOutlierLastDSNum = 0;
    laserCloudSurfTotalLastDSNum = 0;

    potentialLoopFlag = false;
    aLoopIsClosed = false;
    isRefresh = false;
    firstDGO = true;

    latestFrameID = 0;
    Framecnt = 0;

    featureCnt = 0;//xyt 5.27
    featureMatrix = Eigen::MatrixXi::Identity(10000,1+16*I_BIN);//xyt 5.27

}

void mapOptimization::transformAssociateToMap()
{
    float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;

    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
    transformTobeMapped[0] = -asin(srx);

    float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                    - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                    - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                    + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                    + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                    + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
    float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                    - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                    + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                    + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                    - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                    + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
    transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                    crycrx / cos(transformTobeMapped[0]));
    
    float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                    crzcrx / cos(transformTobeMapped[0]));

    x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
    y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];

    x2 = x1;
    y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    transformTobeMapped[3] = transformAftMapped[3] 
                            - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
    transformTobeMapped[4] = transformAftMapped[4] - y2;
    transformTobeMapped[5] = transformAftMapped[5] 
                            - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void mapOptimization::transformUpdate()
{
    if (imuPointerLast >= 0) {
        float imuRollLast = 0, imuPitchLast = 0;
        while (imuPointerFront != imuPointerLast) {
            if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
                break;
            }
            imuPointerFront = (imuPointerFront + 1) % imuQueLength;
        }

        if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
            imuRollLast = imuRoll[imuPointerFront];
            imuPitchLast = imuPitch[imuPointerFront];
        } else {
            int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
            float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) 
                                / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
                            / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

            imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
            imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
        }

        transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
        transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
        }

    for (int i = 0; i < 6; i++) {
        transformBefMapped[i] = transformSum[i];
        transformAftMapped[i] = transformTobeMapped[i];
    }
}

void mapOptimization::updatePointAssociateToMapSinCos(){
    cRoll = cos(transformTobeMapped[0]);
    sRoll = sin(transformTobeMapped[0]);

    cPitch = cos(transformTobeMapped[1]);
    sPitch = sin(transformTobeMapped[1]);

    cYaw = cos(transformTobeMapped[2]);
    sYaw = sin(transformTobeMapped[2]);

    tX = transformTobeMapped[3];
    tY = transformTobeMapped[4];
    tZ = transformTobeMapped[5];
}

void mapOptimization::pointAssociateToMap(PointType const * const pi, PointType * const po)
{
    float x1 = cYaw * pi->x - sYaw * pi->y;
    float y1 = sYaw * pi->x + cYaw * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cRoll * y1 - sRoll * z1;
    float z2 = sRoll * y1 + cRoll * z1;

    po->x = cPitch * x2 + sPitch * z2 + tX;
    po->y = y2 + tY;
    po->z = -sPitch * x2 + cPitch * z2 + tZ;
    po->intensity = pi->intensity;
}

void mapOptimization::updateTransformPointCloudSinCos(PointTypePose *tIn){

    ctRoll = cos(tIn->roll);
    stRoll = sin(tIn->roll);

    ctPitch = cos(tIn->pitch);
    stPitch = sin(tIn->pitch);

    ctYaw = cos(tIn->yaw);
    stYaw = sin(tIn->yaw);

    tInX = tIn->x;
    tInY = tIn->y;
    tInZ = tIn->z;
}

pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn){

    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];
        float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
        float y1 = stYaw * pointFrom->x + ctYaw* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = ctRoll * y1 - stRoll * z1;
        float z2 = stRoll * y1 + ctRoll* z1;

        pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
        pointTo.y = y2 + tInY;
        pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){

    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    
    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];
        float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
        float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
        float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

        pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
        pointTo.y = y2 + transformIn->y;
        pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

void mapOptimization::laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
    timeLaserCloudOutlierLast = msg->header.stamp.toSec();
    laserCloudOutlierLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudOutlierLast);
    newLaserCloudOutlierLast = true;
}

void mapOptimization::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
    timeLaserCloudCornerLast = msg->header.stamp.toSec();
    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudCornerLast);
    newLaserCloudCornerLast = true;
}

void mapOptimization::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
    timeLaserCloudSurfLast = msg->header.stamp.toSec();
    laserCloudSurfLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudSurfLast);
    newLaserCloudSurfLast = true;
}

void mapOptimization::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
    timeLaserOdometry = laserOdometry->header.stamp.toSec();
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;
    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;
    newLaserOdometry = true;
}

void mapOptimization::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;
    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
}

void mapOptimization::laserCloudRawLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){//xyt 5.27
    timeLaserCloudRawLast = msg->header.stamp.toSec();
    laserCloudRawLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudRawLast);

    std::vector<int> remapping;
    pcl::removeNaNFromPointCloud(*laserCloudRawLast, *laserCloudRawLast, remapping);//11.30

    newLaserCloudRawLast = true;
}

void mapOptimization::laserCloudSegmentLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){//xyt 5.27
    timeLaserCloudSegmentLast = msg->header.stamp.toSec();
    laserCloudSegmentLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudSegmentLast);
    newLaserCloudSegmentLast = true;
}


void mapOptimization::publishTF(){

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

    odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
    odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
    odomAftMapped.pose.pose.orientation.z = geoQuat.x;
    odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    odomAftMapped.pose.pose.position.x = transformAftMapped[3];
    odomAftMapped.pose.pose.position.y = transformAftMapped[4];
    odomAftMapped.pose.pose.position.z = transformAftMapped[5];
    odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
    odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
    odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
    odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
    odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
    odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
    pubOdomAftMapped.publish(odomAftMapped);

    aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
    aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
    tfBroadcaster.sendTransform(aftMappedTrans);
}

void mapOptimization::publishKeyPosesAndFrames(){

    if (pubKeyPoses.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = rootnsFrameIdStr+"/camera_init";
        pubKeyPoses.publish(cloudMsgTemp);
    }

    if (pubRecentKeyFrames.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = rootnsFrameIdStr+"/camera_init";
        pubRecentKeyFrames.publish(cloudMsgTemp);
    }
}

//转到raw lidar数据的坐标系下
void mapOptimization::adjustCoordinate(PointTypePose* transformIn, PointTypePose* transformOut){
    transformOut->x = transformIn->z;
    transformOut->y = transformIn->x;
    transformOut->z = transformIn->y;
    transformOut->intensity = transformIn->intensity;
    transformOut->roll  = transformIn->yaw;
    transformOut->pitch = transformIn->roll;
    transformOut->yaw   = transformIn->pitch;
}

void mapOptimization::featureMatrixThread(){//xyt 5.27
    ros::Rate rate(2);
    while (ros::ok()){
        rate.sleep();
        // double start,end;
        // start = clock();
        computeFeatureMatrix();
        // end = clock();
        // std::cout<<"caculate_feature_time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;

    }
}

void mapOptimization::computeFeatureMatrix(){//xyt 5.27
    if(recentRawCloudKeyFrames.size() < delight_accum_frames) return;

    recentRawCloudKeyFrames_mtx.lock();
    for (int i = 0; i < delight_accum_frames ; ++i){
        PointTypePose thisPose6D;
        adjustCoordinate(&cloudKeyPoses6D->points[recentRawCloudKeyFrames[i].first], &thisPose6D);
        *submapKeyFrames += *transformPointCloud(recentRawCloudKeyFrames[i].second, &thisPose6D);
    }
    int thisFrameId = recentRawCloudKeyFrames[delight_accum_frames-1].first;
    recentRawCloudKeyFrames_mtx.unlock();

    // downSizeFilterSubmapKeyPoses.setInputCloud(submapKeyFrames);
    // downSizeFilterSubmapKeyPoses.filter(*submapKeyFramesDS);
    Eigen::MatrixXi descriptor = delight::computeDelightDescriptor(*submapKeyFrames);
    if(descriptor.rows()!=1 || descriptor.cols()!=16*I_BIN){
        ROS_INFO("something weired happen!");
        return;
    }
    featureMatrix.block<1, 16*I_BIN>(_featureCnt,1) = descriptor;
    featureMatrix(_featureCnt,0) = thisFrameId;
    featureCnt++;
    submapKeyFrames->clear();
    // submapKeyFramesDS->clear();

}

void mapOptimization::visualizeGlobalMapThread(){
    ros::Rate rate(0.2);
    while (ros::ok()){
        rate.sleep();
        publishGlobalMap();
    }
}

void mapOptimization::publishGlobalMap(){

    if (pubLaserCloudSurround.getNumSubscribers() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;

    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

    for (int i = 0; i < globalMapKeyPosesDS->points.size(); ++i){
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],   &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(outlierCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    }

    // pcl::PointCloud<PointType>::Ptr interMapAll(new pcl::PointCloud<PointType>);
    // // for(int i=0; i<nRobots; i++){
    //     for(int j=0; j<interMap.size(); j++){
    //         *interMapAll += *interMap[j];
    //     }
    // // }

    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = rootnsFrameIdStr+"/camera_init";
    pubLaserCloudSurround.publish(cloudMsgTemp);  

    // if(interMapAll->points.size() != 0){
    //     sensor_msgs::PointCloud2 intercloudMsgTemp;
    //     pcl::toROSMsg(*interMapAll, intercloudMsgTemp);
    //     intercloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    //     intercloudMsgTemp.header.frame_id = rootnsFrameIdStr+"/camera_init";
    //     pubintermap.publish(intercloudMsgTemp);  
    // }
    
    if(send_map && sendMapClock++ == 5){
        sendMapClock = 0;
        // MapMessage* mmsg = constructMapMessage(globalMapKeyFramesDS, 0, 0);//打包本地最新地图数据
        MapMessage* mmsg = constructMapMessage(cloudKeyPoses3D, 0, 0);//打包本地最新地图数据
        outMapMsg.push_back(std::make_pair(0, mmsg));
    }

    globalMapKeyPoses->clear();
    globalMapKeyPosesDS->clear();
    globalMapKeyFrames->clear();
    globalMapKeyFramesDS->clear();     
}

void mapOptimization::loopClosureThread(){

    if (loopClosureEnableFlag == false)
        return;

    ros::Rate rate(1);
    while (ros::ok()){
        rate.sleep();
        performLoopClosure();
    }
}

bool mapOptimization::detectLoopClosure(){

    latestSurfKeyFrameCloud->clear();
    nearHistorySurfKeyFrameCloud->clear();
    nearHistorySurfKeyFrameCloudDS->clear();

    std::lock_guard<std::mutex> lock(mtx);

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    closestHistoryFrameID = -1;
    for (int i = 0; i < pointSearchIndLoop.size(); ++i){
        int id = pointSearchIndLoop[i];
        if (abs(cloudKeyPoses6D->points[id].time - timeLaserOdometry) > 30.0){
            closestHistoryFrameID = id;
            break;
        }
    }
    if (closestHistoryFrameID == -1){
        return false;
    }

    latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
    *latestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
    *latestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],   &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);

    pcl::PointCloud<PointType>::Ptr hahaCloud(new pcl::PointCloud<PointType>());
    int cloudSize = latestSurfKeyFrameCloud->points.size();
    for (int i = 0; i < cloudSize; ++i){
        if ((int)latestSurfKeyFrameCloud->points[i].intensity >= 0){
            hahaCloud->push_back(latestSurfKeyFrameCloud->points[i]);
        }
    }
    latestSurfKeyFrameCloud->clear();
    *latestSurfKeyFrameCloud   = *hahaCloud;

    for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j){
        if (closestHistoryFrameID + j < 0 || closestHistoryFrameID + j > latestFrameIDLoopCloure)
            continue;
        *nearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[closestHistoryFrameID+j], &cloudKeyPoses6D->points[closestHistoryFrameID+j]);
        *nearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[closestHistoryFrameID+j],   &cloudKeyPoses6D->points[closestHistoryFrameID+j]);
    }

    downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
    downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);

    if (pubHistoryKeyFrames.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = rootnsFrameIdStr+"/camera_init";
        pubHistoryKeyFrames.publish(cloudMsgTemp);
    }
    return true;
}


void mapOptimization::performLoopClosure(){

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    if (potentialLoopFlag == false){

        if (detectLoopClosure() == true){
            potentialLoopFlag = true;
            timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
        }
        if (potentialLoopFlag == false)
            return;
    }

    potentialLoopFlag = false;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(latestSurfKeyFrameCloud);
    icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    if (pubIcpKeyFrames.getNumSubscribers() != 0){
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud (*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = rootnsFrameIdStr+"/camera_init";
        pubIcpKeyFrames.publish(cloudMsgTemp);
    }   

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionCameraFrame;
    correctionCameraFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
    Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
    Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[closestHistoryFrameID]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    std::lock_guard<std::mutex> lock(mtx);
    NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(Symbol(_robotName,latestFrameIDLoopCloure), Symbol(_robotName,closestHistoryFrameID), poseFrom.between(poseTo), constraintNoise));
    if(last_saved_index == -1){
        distMapperHandler->addFactor(factor);
        // distMapperHandler->optimization_LM();
        aLoopIsClosed = true;
    }
    else{
        distMapperHandler->addFactor_isam(factor);//拷贝值上操作
    }
    // distMapperHandler->optimization_LM();
    // graph_mtx.unlock();
    
}

Pose3 mapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint){
    return Pose3(Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll), double(thisPoint.pitch)),
                        Point3(double(thisPoint.z),   double(thisPoint.x),    double(thisPoint.y)));
}

Eigen::Affine3f mapOptimization::pclPointToAffine3fCameraToLidar(PointTypePose thisPoint){
    return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
}

void mapOptimization::extractSurroundingKeyFrames(){

    if (cloudKeyPoses3D->points.empty() == true)
        return;	
    
    if (loopClosureEnableFlag == true){
        if (recentCornerCloudKeyFrames.size() < surroundingKeyframeSearchNum){
            recentCornerCloudKeyFrames. clear();
            recentSurfCloudKeyFrames.   clear();
            recentOutlierCloudKeyFrames.clear();
            int numPoses = cloudKeyPoses3D->points.size();
            for (int i = numPoses-1; i >= 0; --i){
                int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
                PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                updateTransformPointCloudSinCos(&thisTransformation);
                recentCornerCloudKeyFrames. push_front(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                recentSurfCloudKeyFrames.   push_front(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                recentOutlierCloudKeyFrames.push_front(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                if (recentCornerCloudKeyFrames.size() >= surroundingKeyframeSearchNum)
                    break;
            }
        }else{
            if (latestFrameID != cloudKeyPoses3D->points.size() - 1){

                recentCornerCloudKeyFrames. pop_front();
                recentSurfCloudKeyFrames.   pop_front();
                recentOutlierCloudKeyFrames.pop_front();
                latestFrameID = cloudKeyPoses3D->points.size() - 1;
                PointTypePose thisTransformation = cloudKeyPoses6D->points[latestFrameID];
                updateTransformPointCloudSinCos(&thisTransformation);
                recentCornerCloudKeyFrames. push_back(transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
                recentSurfCloudKeyFrames.   push_back(transformPointCloud(surfCloudKeyFrames[latestFrameID]));
                recentOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
            }
        }

        for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i){
            *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
            *laserCloudSurfFromMap   += *recentSurfCloudKeyFrames[i];
            *laserCloudSurfFromMap   += *recentOutlierCloudKeyFrames[i];
        }
    }else{
        surroundingKeyPoses->clear();
        surroundingKeyPosesDS->clear();

        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);
        for (int i = 0; i < pointSearchInd.size(); ++i)
            surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
        for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i){
            bool existingFlag = false;
            for (int j = 0; j < numSurroundingPosesDS; ++j){
                if (surroundingExistingKeyPosesID[i] == (int)surroundingKeyPosesDS->points[j].intensity){
                    existingFlag = true;
                    break;
                }
            }
            if (existingFlag == false){
                surroundingExistingKeyPosesID.   erase(surroundingExistingKeyPosesID.   begin() + i);
                surroundingCornerCloudKeyFrames. erase(surroundingCornerCloudKeyFrames. begin() + i);
                surroundingSurfCloudKeyFrames.   erase(surroundingSurfCloudKeyFrames.   begin() + i);
                surroundingOutlierCloudKeyFrames.erase(surroundingOutlierCloudKeyFrames.begin() + i);
                --i;
            }
        }

        for (int i = 0; i < numSurroundingPosesDS; ++i) {
            bool existingFlag = false;
            for (auto iter = surroundingExistingKeyPosesID.begin(); iter != surroundingExistingKeyPosesID.end(); ++iter){
                if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity){
                    existingFlag = true;
                    break;
                }
            }
            if (existingFlag == true){
                continue;
            }else{
                int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
                PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                updateTransformPointCloudSinCos(&thisTransformation);
                surroundingExistingKeyPosesID.   push_back(thisKeyInd);
                surroundingCornerCloudKeyFrames. push_back(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                surroundingSurfCloudKeyFrames.   push_back(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
            }
        }

        for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
            *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
            *laserCloudSurfFromMap   += *surroundingSurfCloudKeyFrames[i];
            *laserCloudSurfFromMap   += *surroundingOutlierCloudKeyFrames[i];
        }
    }

    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();

    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
}

void mapOptimization::downsampleCurrentScan(){

    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();

    laserCloudOutlierLastDS->clear();
    downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
    downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
    laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

    laserCloudSurfTotalLast->clear();
    laserCloudSurfTotalLastDS->clear();
    *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
    *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
    downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
    downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
    laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
}

void mapOptimization::cornerOptimization(int iterCount){

    updatePointAssociateToMapSinCos();
    for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
        pointOri = laserCloudCornerLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
        
        if (pointSearchSqDis[4] < 1.0) {
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
            }
            cx /= 5; cy /= 5;  cz /= 5;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
                float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                a22 += ay * ay; a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

            matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

            cv::eigen(matA1, matD1, matV1);

            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1 - 0.9 * fabs(ld2);

                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                coeff.intensity = s * ld2;

                if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }
}

void mapOptimization::surfOptimization(int iterCount){
    updatePointAssociateToMapSinCos();
    for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
        pointOri = laserCloudSurfTotalLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel); 
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0.at<float>(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0.at<float>(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0.at<float>(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }
            cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

            float pa = matX0.at<float>(0, 0);
            float pb = matX0.at<float>(1, 0);
            float pc = matX0.at<float>(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                            pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                            pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                        + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }
}

bool mapOptimization::LMOptimization(int iterCount){
    float srx = sin(transformTobeMapped[0]);
    float crx = cos(transformTobeMapped[0]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[2]);
    float crz = cos(transformTobeMapped[2]);

    int laserCloudSelNum = laserCloudOri->points.size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    for (int i = 0; i < laserCloudSelNum; i++) {
        pointOri = laserCloudOri->points[i];
        coeff = coeffSel->points[i];

        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                    + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x 
                    + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arz;
        matA.at<float>(i, 3) = coeff.x;
        matA.at<float>(i, 4) = coeff.y;
        matA.at<float>(i, 5) = coeff.z;
        matB.at<float>(i, 0) = -coeff.intensity;
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        return true;
    }
    return false;
}

void mapOptimization::scan2MapOptimization(){

    if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {

        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        for (int iterCount = 0; iterCount < 10; iterCount++) {

            laserCloudOri->clear();
            coeffSel->clear();

            cornerOptimization(iterCount);
            surfOptimization(iterCount);

            if (LMOptimization(iterCount) == true)
                break;              
        }

        transformUpdate();
    }
}


void mapOptimization::saveKeyFramesAndFactor(){

    currentRobotPosPoint.x = transformAftMapped[3];
    currentRobotPosPoint.y = transformAftMapped[4];
    currentRobotPosPoint.z = transformAftMapped[5];

    bool saveThisKeyFrame = true;
    if (sqrt((previousRobotPosPoint.x-currentRobotPosPoint.x)*(previousRobotPosPoint.x-currentRobotPosPoint.x)
            +(previousRobotPosPoint.y-currentRobotPosPoint.y)*(previousRobotPosPoint.y-currentRobotPosPoint.y)
            +(previousRobotPosPoint.z-currentRobotPosPoint.z)*(previousRobotPosPoint.z-currentRobotPosPoint.z)) < 0.3){
        saveThisKeyFrame = false;
    }

    if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty())
        return;

    previousRobotPosPoint = currentRobotPosPoint;

    // ROS_INFO("FLAG 1");
    // graph_mtx.lock();
    // ROS_INFO("FLAG 2");
    if (cloudKeyPoses3D->points.empty()){
            // ROS_INFO("FLAG 10");
        // gtSAMgraph.add(PriorFactor<Pose3>(Symbol(idRobot(),0), Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
        //                                                     Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise));
        // initialEstimate.insert(Symbol(idRobot(),0), Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
        //                                         Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
        // if(idRobot() == 0){
        //     distMapperHandler->addPrior(Symbol(_robotName,0), Pose3(Rot3::RzRyRx(0, 0, 0),
        //                                                     Point3(0, 0, 0)), priorNoise );
        //     distMapperHandler->insertValue(Symbol(_robotName,0), Pose3(Rot3::RzRyRx(0, 0, 0),
        //                                             Point3(0, 0, 0)));
        // }else{
            distMapperHandler->addPrior(Symbol(_robotName,0), Pose3(Rot3::RzRyRx(0, 0, 0),
                                                            Point3(0, 0, 0)), priorNoise );
            distMapperHandler->insertValue(Symbol(_robotName,0), Pose3(Rot3::RzRyRx(0, 0, 0),
                                                    Point3(0, 0, 0)));            
        // }

        // distMapperHandler->addPrior(Symbol(_robotName,0), Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
        //                                                    Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise );
        // ROS_INFO("FLAG 11");

        // distMapperHandler->insertValue(Symbol(_robotName,0), Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
        //                                         Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));

        for (int i = 0; i < 6; ++i)
            transformLast[i] = transformTobeMapped[i];
    }
    else{
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                                            Point3(transformLast[5], transformLast[3], transformLast[4]));
        gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                            Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
        // gtSAMgraph.add(BetweenFactor<Pose3>(Symbol(idRobot(),cloudKeyPoses3D->points.size()-1), Symbol(idRobot(),cloudKeyPoses3D->points.size()), poseFrom.between(poseTo), odometryNoise));
        // initialEstimate.insert(Symbol(idRobot(),cloudKeyPoses3D->points.size()), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
        //                                                                     Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
        
        NonlinearFactor::shared_ptr factor( new BetweenFactor<Pose3>(Symbol(_robotName,cloudKeyPoses3D->points.size()-1), 
                                                                    Symbol(_robotName,cloudKeyPoses3D->points.size()), 
                                                                    poseFrom.between(poseTo), 
                                                                    odometryNoise) );
        copy_mtx.lock();

        if(isRefresh){
            distMapperHandler->insertValue(Symbol(_robotName,cloudKeyPoses3D->points.size()), 
                            poseFrom.between(poseTo) * distMapperHandler->estimateAt(Symbol(_robotName,cloudKeyPoses3D->points.size()-1)) );
            distMapperHandler->addFactor(factor);
            aLoopIsClosed = true;
            isRefresh = false;
        }else{
            if(last_saved_index != -1){
                distMapperHandler->insertValue_isam(Symbol(_robotName,cloudKeyPoses3D->points.size()), 
                                Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                    Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
                distMapperHandler->addFactor_isam(factor);
            }else{
                distMapperHandler->insertValue(Symbol(_robotName,cloudKeyPoses3D->points.size()), 
                                Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                    Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
                distMapperHandler->addFactor(factor);
            }
        }

        copy_mtx.unlock();
    }
    
    // ROS_INFO("FLAG 3");
    // isam->update(gtSAMgraph, initialEstimate);
    // isam->update();

    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    // gtSAMgraph.resize(0);
    // initialEstimate.clear();
    if(last_saved_index == -1){//不是在拷贝值上操作
        distMapperHandler->optimization_LM();
        isamCurrentEstimate = distMapperHandler->currentEstimate();

    }else{
        // distMapperHandler->optimization_LM2();
        isamCurrentEstimate = distMapperHandler->currentEstimate_isam();
    }

    // isamCurrentEstimate = isam->calculateEstimate();
    // latestEstimate = isamCurrentEstimate.at<Pose3>(Symbol(idRobot(), isamCurrentEstimate.size()-1));
    // latestEstimate = isamCurrentEstimate.at<Pose3>(Symbol(idRobot(), Framecnt));

    // graph_mtx.unlock();

    latestEstimate = isamCurrentEstimate.at<Pose3>(Symbol(_robotName, isamCurrentEstimate.size()-1));    


    thisPose3D.x = latestEstimate.translation().y();
    thisPose3D.y = latestEstimate.translation().z();
    thisPose3D.z = latestEstimate.translation().x();
    thisPose3D.intensity = cloudKeyPoses3D->points.size();
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll  = latestEstimate.rotation().pitch();
    thisPose6D.pitch = latestEstimate.rotation().yaw();
    thisPose6D.yaw   = latestEstimate.rotation().roll();
    thisPose6D.time = timeLaserOdometry;
    cloudKeyPoses6D->push_back(thisPose6D);



    if (cloudKeyPoses3D->points.size() > 1){
        transformAftMapped[0] = latestEstimate.rotation().pitch();
        transformAftMapped[1] = latestEstimate.rotation().yaw();
        transformAftMapped[2] = latestEstimate.rotation().roll();
        transformAftMapped[3] = latestEstimate.translation().y();
        transformAftMapped[4] = latestEstimate.translation().z();
        transformAftMapped[5] = latestEstimate.translation().x();

        for (int i = 0; i < 6; ++i){
            transformLast[i] = transformAftMapped[i];
            transformTobeMapped[i] = transformAftMapped[i];
        }
    }

    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisRawKeyFrame(new pcl::PointCloud<PointType>());//xyt 5.27
    pcl::PointCloud<PointType>::Ptr thisSegmentKeyFrame(new pcl::PointCloud<PointType>());//xyt 5.27

    pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);
    pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);
    pcl::copyPointCloud(*laserCloudRawLast,    *thisRawKeyFrame);//xyt 5.27
    pcl::copyPointCloud(*laserCloudSegmentLast, *thisSegmentKeyFrame);//xyt 5.27

    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
    segmentCloudKeyFrames.push_back(thisSegmentKeyFrame);//xyt 5.27
    delightCloudKeyFrames.push_back(thisRawKeyFrame);//xyt 12.17

    Framecnt++;

    recentRawCloudKeyFrames_mtx.lock();//xyt 5.27
    pair<int,pcl::PointCloud<PointType>::Ptr> thisRawKeyFrameWithId(cloudKeyPoses3D->size()-1,thisRawKeyFrame);//xyt 5.27
    if(recentRawCloudKeyFrames.size()<delight_accum_frames){//xyt 5.27
        recentRawCloudKeyFrames.push_back(thisRawKeyFrameWithId);
    }else{
        // ROS_INFO("WHIW");
        recentRawCloudKeyFrames.pop_front();
        recentRawCloudKeyFrames.push_back(thisRawKeyFrameWithId);
        // ROS_INFO("WHIW2");
    }
    recentRawCloudKeyFrames_mtx.unlock();//xyt 5.27

}

void mapOptimization::correctPoses(){
    if (aLoopIsClosed == true){
        recentCornerCloudKeyFrames. clear();
        recentSurfCloudKeyFrames.   clear();
        recentOutlierCloudKeyFrames.clear();

        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        // for (int i = 0; i < Framecnt; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(Symbol(_robotName,i)).translation().y();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(Symbol(_robotName,i)).translation().z();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(Symbol(_robotName,i)).translation().x();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(Symbol(_robotName,i)).rotation().pitch();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(Symbol(_robotName,i)).rotation().yaw();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(Symbol(_robotName,i)).rotation().roll();
        }

        ROS_INFO("XYT AGAIN");
        aLoopIsClosed = false;
    }
}

void mapOptimization::clearCloud(){
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();  
    laserCloudCornerFromMapDS->clear();
    laserCloudSurfFromMapDS->clear();   
}

void mapOptimization::run(){

    if (newLaserCloudCornerLast  && std::abs(timeLaserCloudCornerLast  - timeLaserOdometry) < 0.005 &&
        newLaserCloudSurfLast    && std::abs(timeLaserCloudSurfLast    - timeLaserOdometry) < 0.005 &&
        newLaserCloudOutlierLast && std::abs(timeLaserCloudOutlierLast - timeLaserOdometry) < 0.005 &&
        newLaserCloudRawLast && std::abs(timeLaserCloudRawLast - timeLaserOdometry) < 0.005 && //xyt 5.27
        newLaserCloudSegmentLast && std::abs(timeLaserCloudSegmentLast - timeLaserOdometry) < 0.005 && //xyt 5.27
        newLaserOdometry)
    {

        newLaserCloudCornerLast = false; newLaserCloudSurfLast = false; newLaserCloudOutlierLast = false; newLaserOdometry = false;
        newLaserCloudRawLast = false; newLaserCloudSegmentLast = false;//xyt 5.27

        std::lock_guard<std::mutex> lock(mtx);

        if (timeLaserOdometry - timeLastProcessing >= mappingProcessInterval) {

            timeLastProcessing = timeLaserOdometry;

            transformAssociateToMap();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            saveKeyFramesAndFactor();

            correctPoses();

            publishTF();

            publishKeyPosesAndFrames();

            clearCloud();
        }
    }
}

int mapOptimization::lastVertex(){
    return cloudKeyPoses3D->points.size() - 1;
}

int mapOptimization::idRobot(){
    return idRobot;
}

RobotMessage* mapOptimization::createMsgfromCharArray(const char* buffer, size_t size){
    boost::mutex::scoped_lock lockg(graphMutex);

    RobotMessage* msg = factoryHandler->fromCharArray(buffer, size);
    return msg;
}

MapMessage* mapOptimization::constructMapMessage(pcl::PointCloud<PointType>::Ptr pointcloud, int inclosure, int outclosure){
    RobotMessage* msg = factoryHandler->constructMessage(3);
    MapMessage* mmsg = dynamic_cast<MapMessage*>(msg);
    mmsg->setRobotId(idRobot());

    mmsg->inclosure = inclosure;
    mmsg->outclosure = outclosure;
    mmsg->map.reset(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*pointcloud,  *(mmsg->map));
    
    std::cerr << "construct map message" << std::endl;
    return mmsg;
}

//收到地图，则马上publish出来以便可视化
void mapOptimization::addInterRobotData(MapMessage* mmsg){
    // lastOutClosure[mmsg->_robotId] = mmsg->inclosure;
    // interMap[mmsg->_robotId].push_back(mmsg->map);
    // interMap.push_back(mmsg->map);
    // inclosures[mmsg->_robotId].push_back(mmsg->outclosure);

    if(mmsg->map->points.size() != 0){//12.02
        sensor_msgs::PointCloud2 intercloudMsgTemp;
        pcl::toROSMsg(*mmsg->map, intercloudMsgTemp);
        intercloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        intercloudMsgTemp.header.frame_id = rootnsFrameIdStr+"/camera_init";
        pubintermap.publish(intercloudMsgTemp);
    }
}

ComboMessage* mapOptimization::constructComboMessage(){

    if(_featureCnt == 0 || last_saved_index != -1){//dgo过程中，就不开启这个流程了
        return NULL;
    }

    double start,end;
    start = clock();

    RobotMessage* msg = factoryHandler->constructMessage(4);
    ComboMessage* cmsg = dynamic_cast<ComboMessage*>(msg);

    cmsg->setRobotId(idRobot());
    cmsg->nodeId = featureMatrix(_featureCnt-1,0);

    cmsg->descriptor.resize(1, 16*I_BIN);
    cmsg->descriptor.block<1,16*I_BIN>(0,0) = featureMatrix.block<1,16*I_BIN>(_featureCnt-1,1);

    end = clock();
    timePR += (end-start)/CLOCKS_PER_SEC;

    return cmsg;
}

//地点识别找到对应的match candidates，发送RPMsg
void mapOptimization::addInterRobotData(ComboMessage* cmsg){

    if(last_saved_index != -1){//dgo过程中，就不开启这个流程了
        return;
    }    

    if(cmsg->descriptor.cols()!=16*I_BIN){
        ROS_INFO("传输错误！！！");
        return;
    }

    if(_featureCnt == 0){
        return;
    }
    double start,end;
    start = clock();

    std::vector<std::pair<int,double> > matches = delight::findMatches(&_featureMatrix, &cmsg->descriptor, featureCnt);//xyt 06.12

    end = clock();
    timePR += (end-start)/CLOCKS_PER_SEC;
    // std::cout<<"add prmsg time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;
    std::cout<<"delight_score:"<<matches[0].second<<std::endl;
    std::cout<<cmsg->nodeId<<" "<<_featureMatrix(matches[0].first, 0)<<std::endl;

    if(matches[0].second < delight_similarity_threshold){
        std::pair<int,int> candidateIdPair(cmsg->nodeId, featureMatrix(matches[0].first, 0));
        candidateIdPairs.push_back(candidateIdPair);

        if(_candidateIdPairs.size() == 1){
            RPMessage* rmsg = constructRPMessage(_candidateIdPairs);
            if(rmsg != NULL){
                outRPMsg.push_back(rmsg);
            }
            candidateIdPairs.clear();
        }
    }
    ROS_INFO("break1563");
    
}

//发送match candidates对应帧的segment_based描述子
RPMessage* mapOptimization::constructRPMessage(std::vector<std::pair<int,int> > candidateIdPairs){
    // boost::mutex::scoped_lock lockg(graphMutex);

    if(last_saved_index != -1){
        return NULL;
    }

    // if(_sendRPMSG == false){
    //     return NULL;
    // }else{
    //     sendRPMSG = false;
    // }

    double start,end;
    start = clock();

    RobotMessage* msg = factoryHandler->constructMessage(6);
    RPMessage* rmsg = dynamic_cast<RPMessage*>(msg);

    rmsg->setRobotId(idRobot());
    rmsg->setCandidateIdPairs(_candidateIdPairs);


    for(int i=0; i<_candidateIdPairs.size(); i++){

        int thisCandidateFrameId = candidateIdPairs[i].second;

        // pcl::PointCloud<PointType>::Ptr segmentSubmap(new pcl::PointCloud<PointType>);
        // pcl::PointCloud<PointType>::Ptr segmentSubmapDS(new pcl::PointCloud<PointType>);
        // PointTypePose thisPose6D;
        // for (int j = thisCandidateFrameId; j > std::max(0,thisCandidateFrameId-5); --j){
        //     adjustCoordinate(&cloudKeyPoses6D->points[j], &thisPose6D);
        //     *segmentSubmap += *transformPointCloud(segmentCloudKeyFrames[j], &thisPose6D);//全局坐标系
        // }
        // if(segmentSubmap->size()==0)return NULL;
        // //转到thisCandidateFrameId坐标系下
        // adjustCoordinate(&cloudKeyPoses6D->points[thisCandidateFrameId], &thisPose6D);
        // Eigen::Isometry3f transToThisFrame = Eigen::Isometry3f::Identity();
        // Eigen::Matrix3f R_transToThisFrame = Eigen::Matrix3f::Identity();
        // R_transToThisFrame = Eigen::AngleAxisf(thisPose6D.yaw, Eigen::Vector3f::UnitZ())
        //                             * Eigen::AngleAxisf(thisPose6D.pitch, Eigen::Vector3f::UnitY())
        //                             * Eigen::AngleAxisf(thisPose6D.roll, Eigen::Vector3f::UnitX());
        // transToThisFrame.rotate( R_transToThisFrame);
        // transToThisFrame.pretranslate( Eigen::Vector3f(thisPose6D.x, thisPose6D.y, thisPose6D.z) );
        // pcl::transformPointCloud(*segmentSubmap, *segmentSubmap, transToThisFrame.inverse());
        

        // downSizeFilterSubmapKeyPoses.setInputCloud(segmentSubmap);
        // downSizeFilterSubmapKeyPoses.filter(*segmentSubmapDS);
            // pcl::io::savePCDFileASCII("/home/xyt/pcd/second"+
            //                             boost::lexical_cast<string>(_candidateIdPairs[i].first)+"_"+
            //                             boost::lexical_cast<string>(_candidateIdPairs[i].second)+".pcd", 
            //                             *segmentCloudKeyFrames[thisCandidateFrameId]);
        EigenFeatures eigenFeatures(segmentCloudKeyFrames[thisCandidateFrameId], segmentation_mode);//
        // EigenFeatures eigenFeatures(segmentSubmapDS, segmentation_mode);//
        eigenFeatures.computeFeatures();

        rmsg->candidates.push_back(eigenFeatures.get_featureMatrix());//候选帧的frame_id和segment_eigenvalues描述子
        rmsg->candidateCentroids.push_back(eigenFeatures.get_segment_centroids());
    }
    
    end = clock();
    timeRP += (end-start)/CLOCKS_PER_SEC;
    std::cout<<"construct rpmsg time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;
    
    return rmsg;
}


//基于segment_based描述子计算match candidates的相对位姿，更新DGO内容
void mapOptimization::addInterRobotData(RPMessage* rmsg){
    if(last_saved_index != -1){
        return;
    }

    double start,end;
    start = clock();

    Eigen::Matrix4f RelativePose = Eigen::Matrix4f::Zero();
    ROS_INFO("break1827");
    for(int i=0; i<rmsg->_candidateIdPairs.size(); i++){

        int thisCandidateFrameId = rmsg->candidateIdPairs[i].first;
        if(thisCandidateFrameId > segmentCloudKeyFrames.size()-1){
            return;
        }

        // //累几帧计算
        // pcl::PointCloud<PointType>::Ptr segmentSubmap(new pcl::PointCloud<PointType>);
        // pcl::PointCloud<PointType>::Ptr segmentSubmapDS(new pcl::PointCloud<PointType>);
        // PointTypePose thisPose6D;
        // for (int j = thisCandidateFrameId; j > std::max(0,thisCandidateFrameId-5); --j){
        //     adjustCoordinate(&cloudKeyPoses6D->points[j], &thisPose6D);
        //     *segmentSubmap += *transformPointCloud(segmentCloudKeyFrames[j], &thisPose6D);//全局坐标系
        // }
        // if(segmentSubmap->size()==0)return;
        // //转到thisCandidateFrameId坐标系下
        // adjustCoordinate(&cloudKeyPoses6D->points[thisCandidateFrameId], &thisPose6D);
        // Eigen::Isometry3f transToThisFrame = Eigen::Isometry3f::Identity();
        // Eigen::Matrix3f R_transToThisFrame = Eigen::Matrix3f::Identity();
        // R_transToThisFrame = Eigen::AngleAxisf(thisPose6D.yaw, Eigen::Vector3f::UnitZ())
        //                             * Eigen::AngleAxisf(thisPose6D.pitch, Eigen::Vector3f::UnitY())
        //                             * Eigen::AngleAxisf(thisPose6D.roll, Eigen::Vector3f::UnitX());
        // transToThisFrame.rotate( R_transToThisFrame);
        // transToThisFrame.pretranslate( Eigen::Vector3f(thisPose6D.x, thisPose6D.y, thisPose6D.z) );
        // pcl::transformPointCloud(*segmentSubmap, *segmentSubmap, transToThisFrame.inverse());
        

        // downSizeFilterSubmapKeyPoses.setInputCloud(segmentSubmap);
        // downSizeFilterSubmapKeyPoses.filter(*segmentSubmapDS);
            // pcl::io::savePCDFileASCII("/home/xyt/pcd/first"+
            //                             boost::lexical_cast<string>(rmsg->candidateIdPairs[i].first)+"_"+
            //                             boost::lexical_cast<string>(rmsg->candidateIdPairs[i].second)+".pcd", 
            //                             *segmentCloudKeyFrames[thisCandidateFrameId]);

        EigenFeatures eigenFeatures(segmentCloudKeyFrames[thisCandidateFrameId], segmentation_mode);
        // EigenFeatures eigenFeatures(segmentSubmapDS, segmentation_mode);//
        eigenFeatures.computeFeatures();

        //计算相对位姿 candidateIdPairs[i].first=>_candidateIdPairs[i].second
        // start = clock();
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        pcl::PointCloud<PclPoint>::Ptr first_cloud(new pcl::PointCloud<PclPoint>());
        pcl::PointCloud<PclPoint>::Ptr second_cloud(new pcl::PointCloud<PclPoint>());
        eigenFeatures.findMatches(rmsg->candidates[i], rmsg->candidateCentroids[i], 
                                    correspondences, first_cloud, second_cloud);
        // end = clock();
        // std::cout<<"match_time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;

        //几何一致性计算，得到transform
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > correspondence_transformations;
        std::vector<pcl::Correspondences> clustered_corrs;

        double start,end;
        start = clock();
        pcl::GeometricConsistencyGrouping<PclPoint, PclPoint> geometric_consistency_grouping;
        geometric_consistency_grouping.setGCSize(gc_resolution);
        geometric_consistency_grouping.setGCThreshold(gc_min_cluster_size - 1);
        geometric_consistency_grouping.setInputCloud(first_cloud);
        geometric_consistency_grouping.setSceneCloud(second_cloud);

        geometric_consistency_grouping.setModelSceneCorrespondences(correspondences);
        geometric_consistency_grouping.recognize(correspondence_transformations, clustered_corrs);
        end = clock();
        std::cout<<"gc_time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;

        for(int trans_cnt=0; trans_cnt<correspondence_transformations.size(); trans_cnt++){
            if(correspondence_transformations[trans_cnt] != Eigen::Matrix4f::Identity()){
                RelativePose = correspondence_transformations[trans_cnt]/rmsg->candidateIdPairs.size();
                break;
            }
        }
    }

    end = clock();
    timeRP += (end-start)/CLOCKS_PER_SEC;
    // std::cout<<"add rpmsg time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;

    if(RelativePose(3,3) == 0){
        ROS_INFO("Reject one candidate match!");
        // sendPRMSG = true;
        return;
    }//没有计算出transformation，则直接返回

    // sendRPMSG = false;

    Eigen::Vector3f ea = RelativePose.block<3,3>(0,0).eulerAngles(0,1,2);
    gtsam::Pose3 edgePose = Pose3(Rot3::RzRyRx(double(ea[0]), double(ea[1]), double(ea[2])), 
                                Point3(double(RelativePose(0,3)), double(RelativePose(1,3)), double(RelativePose(2,3))));//

    // graph_mtx.lock();
    DistGraphMessage* dgmsg;

    if(rmsg->candidateIdPairs.size() > 1){
       NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(Symbol(_robotName, 0),
                                                                    Symbol(ROBOT_NAMES[rmsg->_robotId], 0), 
                                                                    edgePose, 
                                                                    outerEdgeNoise));
        distMapperHandler->addFactor(factor);
        distMapperHandler->updateNeighbor(Symbol(ROBOT_NAMES[rmsg->_robotId], 0), Pose3());
        lastSepNode[rmsg->_robotId] = rmsg->candidateIdPairs[0].second;//记录上一个sep node

        dgmsg = constructDistGraphMessage(false, factor, rmsg->_robotId);

        // if(_distMapperHandler->seperatorEdge().size() % 2 == 0){
        //     return;
        // }
        // dgmsg = constructDistGraphMessage(false, factor, rmsg->_robotId);

        if(_distMapperHandler->seperatorEdge().size() % 2 != 0){
            iterCnt = 0;//有新边，重新启动迭代过程
            firstStage = true;
            // if(last_saved_index != -1){
            //     distMapperHandler->insertValue(last_saved_index);//copy值更新到graph上
            // }
            copy_mtx.lock();
            distMapperHandler->create_copy();
            last_saved_index = distMapperHandler->currentEstimate().size()-1;//在copy上操作
            // if(_firstDGO){
                distMapperHandler->removePrior();
            // }
            copy_mtx.unlock();

            if(idRobot() == 0){
                distMapperHandler->estimateRotation();
                // distMapperHandler->updateInitialized(true);//=====note
                distMapperHandler->updateRotation();        
            }
            
            // dgmsg = constructDistGraphMessage(false, factor, rmsg->_robotId);
        }


    }else{
        NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(Symbol(_robotName, rmsg->candidateIdPairs[0].first),
                                                                    Symbol(ROBOT_NAMES[rmsg->_robotId], rmsg->candidateIdPairs[0].second), 
                                                                    edgePose, 
                                                                    outerEdgeNoise));
        distMapperHandler->addFactor(factor);
        distMapperHandler->updateNeighbor(Symbol(ROBOT_NAMES[rmsg->_robotId], rmsg->candidateIdPairs[0].second), Pose3());
        std::cout<<"----addfactor"<<rmsg->candidateIdPairs[0].first<<" "<<rmsg->candidateIdPairs[0].second<<std::endl;
        lastSepNode[rmsg->_robotId] = rmsg->candidateIdPairs[0].second;

        dgmsg = constructDistGraphMessage(false, factor, rmsg->_robotId);

        // if(_distMapperHandler->seperatorEdge().size() % 2 == 0){
        //     return;
        // }
        // dgmsg = constructDistGraphMessage(false, factor, rmsg->_robotId);

        if(_distMapperHandler->seperatorEdge().size() % 2 != 0){
            iterCnt = 0;//有新边，重新启动迭代过程
            firstStage = true;
            // if(last_saved_index != -1){
            //     distMapperHandler->insertValue(last_saved_index);//copy值更新到graph上
            // }
            copy_mtx.lock();
            distMapperHandler->create_copy();
            last_saved_index = distMapperHandler->currentEstimate().size()-1;//在copy上操作
            // if(_firstDGO){
                distMapperHandler->removePrior();
            // }
            copy_mtx.unlock();

            // if(idRobot() == 0){
            //     distMapperHandler->estimateRotation();
            //     // distMapperHandler->updateInitialized(true);
            //     distMapperHandler->updateRotation();        
            // }
        }  
    }
    // graph_mtx.unlock();


    // lastSepNode[rmsg->_robotId] = rmsg->candidateIdPairs[0].second;

    if(dgmsg!=NULL){
        outDistGraphMsg.push_back(dgmsg);
    }

}

DistGraphMessage* mapOptimization::constructDistGraphMessage(bool fin, NonlinearFactor::shared_ptr factor, int to_robot_id){

    double start,end;
    start = clock();

    RobotMessage* msg = factoryHandler->constructMessage(7);
    DistGraphMessage* dgmsg = dynamic_cast<DistGraphMessage*>(msg);
    dgmsg->setRobotId(idRobot());

    dgmsg->is_init = distMapperHandler->isRobotInitialized() ? 1 : -1;

    if(fin){//终止信号
        dgmsg->has_edge = -2;//标记       
    }
    else if(factor != NULL){
        dgmsg->has_edge = 1;

        BetweenFactor<Pose3>::shared_ptr edge = dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);

        if(edge){
            ESE3Data *temp = new ESE3Data();
            temp->idfrom = Symbol(edge->key1()).index();
            temp->idto = Symbol(edge->key2()).index();
            temp->inter = to_robot_id;
            temp->estimate[0] = edge->measured().translation().x();
            temp->estimate[1] = edge->measured().translation().y();
            temp->estimate[2] = edge->measured().translation().z();
            temp->estimate[3] = edge->measured().rotation().roll();
            temp->estimate[4] = edge->measured().rotation().pitch();
            temp->estimate[5] = edge->measured().rotation().yaw();

            SharedNoiseModel model = edge->noiseModel();
            noiseModel::Gaussian::shared_ptr gaussianModel = dynamic_pointer_cast<noiseModel::Gaussian>(model);
            Matrix info = gaussianModel->R().transpose() * gaussianModel->R();   

            int k = 0;
            for(int m=0;m<6;m++){
                for(int n=0;n<6;n++){
                    temp->information[k++] = info(m,n);
                }
            }
            dgmsg->edgeVector.push_back(*temp);
        }else{
            std::cerr << "something wrong" << std::endl;
        }
    }
    else{
        dgmsg->has_edge = -1;
    }

    std::vector<size_t> sepEdgeIds = distMapperHandler->seperatorEdge();
    for(size_t sepSlot : sepEdgeIds){
        boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
            boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(_distMapperHandler->currentGraph().at(sepSlot));
        // Construct between chordal factor corresponding to separator edges
        KeyVector keys = pose3Between->keys();
        Symbol key0 = keys.at(0);
        Symbol key1 = keys.at(1);
        char robot0 = symbolChr(key0);
        char robot1 = symbolChr(key1);
        if(robot0 == robotName){
            if(!dgmsg->linearize_rotations.exists(key0)){
                dgmsg->linearize_rotations.insert(key0, distMapperHandler->linearizedRotationAt(key0));
                if(_distMapperHandler->linearizedPoses().exists(key0)){
                        dgmsg->linearize_poses.insert(key0, distMapperHandler->linearizedPosesAt(key0));
                }
            }
        }else if(robot1 == robotName){
            if(!dgmsg->linearize_rotations.exists(key1)){
                dgmsg->linearize_rotations.insert(key1, distMapperHandler->linearizedRotationAt(key1));
                if(_distMapperHandler->linearizedPoses().exists(key1)){
                        dgmsg->linearize_poses.insert(key1, distMapperHandler->linearizedPosesAt(key1));
                }
            }
        }else{
            std::cerr <<"something wrong" <<std::endl;
        }
    }    

    end = clock();
    timeDGO += (end-start)/CLOCKS_PER_SEC;
    std::cout<<"add dgomsg time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;

    return dgmsg;
}

void mapOptimization::addInterRobotData(DistGraphMessage* dgmsg){

    double start,end;
    start = clock();

    if(dgmsg->has_edge > 0  /*&& last_saved_index == -1 */){
        std::cerr << dgmsg->edgeVector.size()<<"|"<<dgmsg->edgeVector[0].inter<<std::endl;
        if(dgmsg->edgeVector.size() != 0 && dgmsg->edgeVector[0].inter == idRobot()){//需保证edge为本机参与的边
            Point3 t(dgmsg->edgeVector[0].estimate[0],dgmsg->edgeVector[0].estimate[1],dgmsg->edgeVector[0].estimate[2]);
            Rot3 R = Rot3::RzRyRx(dgmsg->edgeVector[0].estimate[3],dgmsg->edgeVector[0].estimate[4],dgmsg->edgeVector[0].estimate[5]);
            Matrix info_m = I_6x6;
            int k = 0;
            for(int m=0;m<6;m++){
                for(int n=0;n<6;n++){
                    info_m(m,n) = dgmsg->edgeVector[0].information[k++];
                }
            }            
            SharedNoiseModel model =  noiseModel::Gaussian::Information(info_m);

            NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(Symbol(ROBOT_NAMES[dgmsg->_robotId],dgmsg->edgeVector[0].idfrom),Symbol(_robotName,dgmsg->edgeVector[0].idto),Pose3(R,t),outerEdgeNoise));
            
            // graph_mtx.lock();
            distMapperHandler->addFactor(factor);
            distMapperHandler->updateNeighbor(Symbol(ROBOT_NAMES[dgmsg->_robotId],dgmsg->edgeVector[0].idfrom), Pose3());
            lastSepNode[dgmsg->_robotId] = dgmsg->edgeVector[0].idto;
            std::cout<<"----addfactor"<<dgmsg->edgeVector[0].idfrom<<" "<<dgmsg->edgeVector[0].idto<<std::endl;

            if(_distMapperHandler->seperatorEdge().size() % 2 == 0){
                return;
            }
            graph_mtx.unlock();

            iterCnt = 0;//新边，重启迭代
            firstStage = true;
            if(last_saved_index == -1){
                copy_mtx.lock();
                distMapperHandler->create_copy();

                last_saved_index = distMapperHandler->currentEstimate().size()-1;//在copy上操作
                // if(_firstDGO){
                    distMapperHandler->removePrior();
                // }
                copy_mtx.unlock();
            }else{//两边同时发则都终止
                if(idRobot() > dgmsg->_robotId){//序号较大的重新发起dgo
                    dgmsg = constructDistGraphMessage(dgmsg->_robotId);
                    if(dgmsg!=NULL){
                        outDistGraphMsg.push_back(dgmsg);
                    }
                }
                return;
            }
        }
    }


    if(_iterCnt < distMapperHandlers_max_iter){
    
        if(_firstStage){
            for(gtsam::VectorValues::KeyValuePair &key_value : dgmsg->linearize_rotations){
                gtsam::Key key = key_value.first;
                distMapperHandler->updateNeighborLinearizedRotations(key, dgmsg->linearize_rotations.at(key));
            }
        }else{
            for(gtsam::VectorValues::KeyValuePair &key_value : dgmsg->linearize_poses){
                gtsam::Key key = key_value.first;
                distMapperHandler->updateNeighborLinearizedPoses(key, dgmsg->linearize_poses.at(key));
            }
        }

        if(_iterCnt == 0){//每个阶段刚开始检查是否初始化
            if(dgmsg->is_init > 0){
                distMapperHandler->updateNeighboringRobotInitialized(ROBOT_NAMES[dgmsg->_robotId], true);
            }else{//对方未初始化-》确保本机初始化，发消息给对方
                distMapperHandler->updateNeighboringRobotInitialized(ROBOT_NAMES[dgmsg->_robotId], false);
                if(idRobot() != 0){
                    DistGraphMessage* new_dgmsg= constructDistGraphMessage();//第一阶段结束，继续发消息
                    outDistGraphMsg.push_back(new_dgmsg);                    
                    return;
                }
                // distMapperHandler->updateInitialized(true);
               
                // if(_distMapperHandler->isRobotInitialized()){
                //     DistGraphMessage* new_dgmsg= constructDistGraphMessage();
                //     outDistGraphMsg.push_back(new_dgmsg);
                // }

                // return;
            }
        }else{
            distMapperHandler->updateNeighboringRobotInitialized(ROBOT_NAMES[dgmsg->_robotId], true);
        }

        mtx.lock();

        if(_firstStage){
            distMapperHandler->estimateRotation();
            distMapperHandler->updateInitialized(true);
            distMapperHandler->updateRotation();
        }else{
            distMapperHandler->estimatePoses();
            distMapperHandler->updateInitialized(true);
            distMapperHandler->updatePoses();
        }

        iterCnt++;

        std::cout<<"lastest_change:--------"<<_distMapperHandler->latestChange()<<std::endl; 
        if(_distMapperHandler->latestChange() < poseEstimateChangeThreshold  || iterCnt == distMapperHandlers_max_iter || dgmsg->has_edge == -2){//收敛，不再发送sep_node消息
            
            if(_firstStage){
                iterCnt = 0;

                distMapperHandler->convertLinearizedRotationToPoses();
                for(const gtsam::Values::ConstKeyValuePair& key_value : distMapperHandler->neighbors()){
                    gtsam::Key key = key_value.key;
                    gtsam::VectorValues linRotEstimateNeighbor;
                    linRotEstimateNeighbor.insert( key,  distMapperHandler->neighborsLinearizedRotationsAt(key));
                    // make a pose out of it
                    gtsam::Values rotEstimateNeighbor = gtsam::InitializePose3::normalizeRelaxedRotations(linRotEstimateNeighbor);
                    gtsam::Values poseEstimateNeighbor = multirobot_util::pose3WithZeroTranslation(rotEstimateNeighbor);
                    // store it
                    distMapperHandler->updateNeighbor(key, poseEstimateNeighbor.at<gtsam::Pose3>(key));
                }
                std::cout<<_distMapperHandler->linearizedPoses().size()<<std::endl;

                firstStage = false;

                // if(idRobot() != 0){
                    distMapperHandler->updateInitialized(false);
                // }else{
                //     // distMapperHandler->estimatePoses();
                //     // distMapperHandler->updatePoses();                    
                // }
                distMapperHandler->clearNeighboringRobotInit();


                DistGraphMessage* new_dgmsg= constructDistGraphMessage();//第一阶段结束，继续发消息
                outDistGraphMsg.push_back(new_dgmsg);
                
                ROS_INFO("Stage1 COMPLETE!!!");

            }else{
                iterCnt = distMapperHandlers_max_iter;

                if(dgmsg->has_edge != -2){//对方未终止迭代，继续发送消息
                    DistGraphMessage* new_dgmsg= constructDistGraphMessage(true);//终止信号
                    outDistGraphMsg.push_back(new_dgmsg);                    
                }

                // if(_distMapperHandler->latestChange() < poseEstimateChangeThreshold){
                //     distMapperHandler->retractPose3Global();
                //     distMapperHandler->update_priors();
                //     distMapperHandler->seperatorEdge().pop_back();
                // }

                distMapperHandler->retractPose3Global();
                distMapperHandler->update_priors();

                firstStage = true;
                ROS_INFO("DGO COMPLETE!!!");
                
                copy_mtx.lock();
                distMapperHandler->addValuesFromCopy(last_saved_index);
                distMapperHandler->addFactorFromCopy(last_saved_index);
                last_saved_index = -1;
                isRefresh = true;
                firstDGO = false;
                copy_mtx.unlock();

                // aLoopIsClosed = true;

                // if(idRobot() != 0){
                    distMapperHandler->updateInitialized(false);
                // }
                distMapperHandler->clearNeighboringRobotInit();
                
            }

        }else{
            DistGraphMessage* new_dgmsg= constructDistGraphMessage();
            outDistGraphMsg.push_back(new_dgmsg);
        }

        // std::cerr<<"prior0->>"<<_distMapperHandler->estimateAt(Symbol(_robotName, 0)).matrix()<<std::endl;

        // std::cerr<<_distMapperHandler->currentEstimate().size()<<"|"<<_distMapperHandler->innerEdges().size()<<std::endl;
        // distMapperHandler->optimization_LM();

        // isamCurrentEstimate = distMapperHandler->currentEstimate();
        // correctPoses();

        // graph_mtx.unlock();
        mtx.unlock();
    }
    // else{//收敛，还继续收到dgo消息
    //     DistGraphMessage* new_dgmsg= constructDistGraphMessage();
    //     outDistGraphMsg.push_back(new_dgmsg);         
    // }

    end = clock();
    timeDGO += (end-start)/CLOCKS_PER_SEC;
    // std::cout<<"add rpmsg time:"<<(end-start)/CLOCKS_PER_SEC<<std::endl;

    // pcl::PointCloud<PointType>::Ptr mapToSend(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr mapToSendDS(new pcl::PointCloud<PointType>());

    // // std::cerr <<"testttttt:"<< lastOutClosuremap[gmsg->_robotId]+1 << " " << gmsg->targetVertex << std::endl;
    // for (int i = lastOutClosuremap[gmsg->_robotId]+1; i <= gmsg->targetVertex; ++i){
    //     *mapToSend += *transformPointCloud(cornerCloudKeyFrames[i],   &cloudKeyPoses6D->points[i]);
    //     *mapToSend += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
    //     *mapToSend += *transformPointCloud(outlierCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    // }
    // lastOutClosuremap[gmsg->_robotId] = gmsg->targetVertex;

    // downSizeFilterGlobalMapKeyFramesoutput.setInputCloud(mapToSend);
    // downSizeFilterGlobalMapKeyFramesoutput.filter(*mapToSendDS);

    // MapMessage* mmsg = constructMapMessage(mapToSendDS, inclosure, outclosure);
    // outMapMsg.push_back(make_pair(gmsg->_robotId, mmsg));
}

void mapOptimization::addInterRobotData(StampedRobotMessage smsg){
    boost::mutex::scoped_lock lockg(graphMutex);
    // std::cerr << "Adding inter robot data " << std::endl;
    ComboMessage* cmsg = dynamic_cast<ComboMessage*>(smsg.msg);
    if (cmsg){
        std::cerr << "Adding PR data " << std::endl;
        addInterRobotData(cmsg);
    }
    else{
        RPMessage* rmsg = dynamic_cast<RPMessage*>(smsg.msg);
        if(rmsg){
            std::cerr << "Adding RP data" << std::endl;
            addInterRobotData(rmsg);
        }
        else{
            DistGraphMessage* dgmsg = dynamic_cast<DistGraphMessage*>(smsg.msg);
            if (dgmsg){
                std::cerr << "Adding DGO data " << std::endl;
                addInterRobotData(dgmsg);
            }
            else{
                MapMessage* mmsg = dynamic_cast<MapMessage*>(smsg.msg);
                if (mmsg){
                    std::cerr << "Adding map data" << std::endl;
                    addInterRobotData(mmsg);
                }
                else{
                    // RobotLaserMessage* rmsg = dynamic_cast<RobotLaserMessage*>(smsg.msg);
                    // if(rmsg)
                    //     addInterRobotData(rmsg);
                    // else
                        /****/
                }
            }
        }

    }
}

bool mapOptimization::outputTraj_service(lego_loam::OutputTrajRequest& req, lego_loam::OutputTrajResponse& res) {

    ROS_INFO("SERVICE CALLED");
    std::string directory = req.destination;

    boost::filesystem::remove_all(directory);
    boost::filesystem::create_directory(directory);

    std::ofstream output_traj_ofs(directory+"/traj.txt");
    std::cout << "result saved to:" << directory << std::endl;

    // graph_slam->save(directory + "/graph.g2o");
    //转到原始lidar数据坐标系的pose输出
    for (int i = 0; i < cloudKeyPoses6D->points.size(); ++i){
        output_traj_ofs << std::setprecision(12)
                        << cloudKeyPoses6D->points[i].time << " "
                        << cloudKeyPoses6D->points[i].x << " "
                        << cloudKeyPoses6D->points[i].y << " "
                        << cloudKeyPoses6D->points[i].z << " "
                        << cloudKeyPoses6D->points[i].roll << " "
                        << cloudKeyPoses6D->points[i].pitch << " "
                        << cloudKeyPoses6D->points[i].yaw << std::endl;
    }

    writeG2o(_distMapperHandler->currentGraph(),_distMapperHandler->currentEstimate(),directory+"/graph.g2o") ;

    res.success = true;
    return true;
}

bool mapOptimization::outputMap_service(lego_loam::OutputMapRequest& req, lego_loam::OutputMapResponse& res){
    std::string directory = req.destination;
    double resolution = req.resolution;

    pcl::PointCloud<PointType>::Ptr outputMap(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr outputMapDS(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> downSizeFilter;

    for (int i = 0; i < cloudKeyPoses6D->points.size(); ++i){
        int thisKeyInd = (int)cloudKeyPoses6D->points[i].intensity;
        *outputMap += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],   &cloudKeyPoses6D->points[thisKeyInd]);
        *outputMap += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        *outputMap += *transformPointCloud(outlierCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    }
    downSizeFilter.setLeafSize(resolution, resolution, resolution);
    downSizeFilter.setInputCloud(outputMap);
    downSizeFilter.filter(*outputMapDS);

    //输出的是转过坐标系的pcd
    pcl::io::savePCDFileASCII(directory+"/map.pcd",*outputMapDS);
    pcl::io::savePCDFileASCII(directory+"/traj.pcd",*cloudKeyPoses3D);

    res.success = true;
    return true;
}

bool mapOptimization::outputSepNodePcd_service(lego_loam::OutputSepNodePcdRequest& req, lego_loam::OutputSepNodePcdResponse& res){
    
    std::string directory = req.destination;

    std::vector<size_t> sepEdgeIds = distMapperHandler->seperatorEdge();
    for(size_t sepSlot : sepEdgeIds){
        boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
            boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(_distMapperHandler->currentGraph().at(sepSlot));
        // Construct between chordal factor corresponding to separator edges
        KeyVector keys = pose3Between->keys();
        Symbol key0 = keys.at(0);
        Symbol key1 = keys.at(1);
        char robot0 = symbolChr(key0);
        char robot1 = symbolChr(key1);
        int index0 = key0.index();
        int index1 = key1.index();
        if(robot0 == robotName){
            pcl::io::savePCDFileASCII(directory+"/pcd/first"+
                                    boost::lexical_cast<string>(index0)+"_"+
                                    boost::lexical_cast<string>(index1)+".pcd", 
                                    *segmentCloudKeyFrames[index0]);
            pcl::io::savePCDFileASCII(directory+"/pcd/first"+
                                    boost::lexical_cast<string>(index0)+"_"+
                                    boost::lexical_cast<string>(index1)+"_delight.pcd", 
                                    *delightCloudKeyFrames[index0]);
        }else if(robot1 == robotName){
            pcl::io::savePCDFileASCII(directory+"/pcd/second"+
                                        boost::lexical_cast<string>(index0)+"_"+
                                        boost::lexical_cast<string>(index1)+".pcd", 
                                        *segmentCloudKeyFrames[index1]);
            pcl::io::savePCDFileASCII(directory+"/pcd/second"+
                                        boost::lexical_cast<string>(index0)+"_"+
                                        boost::lexical_cast<string>(index1)+"_delight.pcd", 
                                        *delightCloudKeyFrames[index1]);
        }else{
            std::cerr <<"something wrong" <<std::endl;
        }
    }  

    //输出参数
    std::ofstream parameters_log;
    parameters_log.open(directory + "/parameters.txt");

    parameters_log<<"I_BIN: "<<I_BIN<<std::endl;
    parameters_log<<"delight_accum_frames: "<<delight_accum_frames<<std::endl;
    parameters_log<<"delight_similarity_threshold: "<<delight_similarity_threshold<<std::endl;
    parameters_log<<"n_nearest_neighbours: "<<n_nearest_neighbours<<std::endl;
    parameters_log<<"feature_distance_threshold: "<<feature_distance_threshold<<std::endl;
    parameters_log<<"gc_resolution: "<<gc_resolution<<std::endl;
    parameters_log<<"gc_min_cluster_size: "<<gc_min_cluster_size<<std::endl;
    parameters_log<<"segmentation_mode: "<<segmentation_mode<<std::endl;
    parameters_log<<"SIM_COMM_RANGE: "<<SIM_COMM_RANGE<<std::endl;
    parameters_log<<"SEPNODE_STEP: "<<SEPNODE_STEP<<std::endl;

    res.success = true;
    return true;
}

