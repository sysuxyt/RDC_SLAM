#ifndef _DELIGHT_H_
#define _DELIGHT_H_

#include "utility.h"

static bool Less(const pair<int,double>& s1, const pair<int,double>& s2){
    return s1.second < s2.second; //从小到大排序
}

/**
 * @name: 
 * @msg:计算pointcloud的delight描述子
 * @param {PointCloud<pcl::PointXYZI>} point_cloud
 * @return {*} 一维向量
 */
Eigen::MatrixXi computeDelightDescriptor(pcl::PointCloud<pcl::PointXYZI> point_cloud){
    // start_time = clock();
    
    double alignment_rad;
    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(point_cloud, pca_centroid);

    Eigen::Affine3f translate = Eigen::Affine3f::Identity();
    translate.translate(Eigen::Vector3f(-pca_centroid[0], -pca_centroid[1], -pca_centroid[2]));
    pcl::transformPointCloud(point_cloud, point_cloud, translate);

    pca_centroid[0] = 0;
    pca_centroid[1] = 0;
    pca_centroid[2] = 0;
    pca_centroid[3] = 1;

    Eigen::Matrix3f covariance_3d;
    computeCovarianceMatrixNormalized(point_cloud, pca_centroid, covariance_3d);
    const Eigen::Matrix2f covariance_2d = covariance_3d.block(0, 0, 2u, 2u);
    Eigen::EigenSolver<Eigen::Matrix2f> eigen_solver(covariance_2d, true);

    alignment_rad = atan2(eigen_solver.eigenvectors()(1,0).real(),
                        eigen_solver.eigenvectors()(0,0).real());

    if (eigen_solver.eigenvalues()(0).real() <
        eigen_solver.eigenvalues()(1).real()) {
    alignment_rad += 0.5*M_PI;
    }

    // Rotate the segment.
    alignment_rad = -alignment_rad;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(alignment_rad, Eigen::Vector3f::UnitZ()));
    pcl::PointCloud<pcl::PointXYZI> rotated_point_cloud;
    pcl::transformPointCloud(point_cloud, rotated_point_cloud, transform);

    // Get most points on the lower half of y axis (by rotation).
    pcl::PointXYZI point_min, point_max;
    pcl::getMinMax3D(rotated_point_cloud, point_min, point_max);
    double centroid_y = point_min.y + (point_max.y - point_min.y) / 2.0;
    unsigned int n_below = 0;
    for (const pcl::PointXYZI point:rotated_point_cloud.points) {
    if (point.y < centroid_y) ++n_below;
    }
    if (static_cast<double>(n_below) < static_cast<double>(rotated_point_cloud.size()) / 2.0) {
    alignment_rad += M_PI;
    Eigen::Affine3f adjustment_transform = Eigen::Affine3f::Identity();
    adjustment_transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(rotated_point_cloud, rotated_point_cloud, adjustment_transform);
    }

    //统计直方图
    Eigen::MatrixXi descriptor(1, 16*I_BIN);
    descriptor.setZero(1, 16*I_BIN);

    for (const pcl::PointXYZI point: rotated_point_cloud.points) {
        int p,q;
        p = std::floor(atan2(point.x, point.y)/(M_PI/4))+4;
        float range = std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
        if(range < inner_sphere_r1){
            q = 0;
        }else if(range < outer_sphere_r2){
            q = 1;
        }
        else{
            continue;
        }
        // if(point.intensity>=64){
        //   descriptor[(q*8+p)*I_BIN+I_BIN-1]++;
        // }else{
            // descriptor[(q*8+p)*I_BIN+std::floor(point.intensity*I_BIN/256)]++;
            descriptor(0, (q*8+p)*I_BIN+std::floor(point.intensity*I_BIN/256))++;
        // }
        // if(point.intensity>128) intensity_cnt++;
    }

    // end_time = clock();
    // std::cout<<(end_time-start_time)/CLOCKS_PER_SEC<<std::endl;
    return descriptor;
    
}

/**
 * @name: 
 * @msg:计算两个描述子的匹配分数
 * @param {MatrixXi} descriptorA
 * @param {MatrixXi} descriptorB
 * @return {*}
 */
double matchDescriptor(Eigen::MatrixXi descriptorA, Eigen::MatrixXi descriptorB){
    vector<float> dist_bin;
    dist_bin.resize(16);
    double dist_mean = 0;
    for(int i=0; i<16; i++){
        dist_bin[i] = 0;
        for(int j=0; j<I_BIN; j++){
            double Ak = descriptorA(0,i*I_BIN+j);
            double Bk = descriptorB(0,i*I_BIN+j);
            if((Ak+Bk)!=0){
                dist_bin[i] += 2*std::pow((Ak-Bk),2)/(Ak+Bk);
                // dist_bin[i] += std::abs(Ak-Bk)/(Ak+Bk);
            }
        }
        dist_mean += dist_bin[i]/16;
    }
    return dist_mean;
}

/**
 * @name: 
 * @msg:一对多，查找匹配分数最高的描述子
 * @param {const int} target_num
 * @return｛*}　　
 */
vector<pair<int,double> > findMatches(Eigen::MatrixXi* target_matrix, Eigen::MatrixXi* source_feature, const int target_num){
//   vector<pair<int,double> > dists(target_matrix->rows());
    vector<pair<int,double> > dists(target_num);
    for(int row=0; row<target_num; row++){
        pair<int,double> dist(row, 
                matchDescriptor(target_matrix->block<1,16*I_BIN>(row,1), source_feature->block<1,16*I_BIN>(0,0)));
        dists[row] = dist;
    }
    sort(dists.begin(),dists.end(),Less);
    return dists;
}


#endif