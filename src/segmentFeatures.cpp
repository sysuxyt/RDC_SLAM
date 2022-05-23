#ifndef SEGMENT_FEATURES_H
#define SEGMENT_FEATURES_H

#include "segmentFeatures.h"

SegmentFeatures::SegmentFeatures(std::vector<pcl::PointCloud<PointType>::Ptr> segments){
  segments = segments;
}

SegmentFeatures::SegmentFeatures(pcl::PointCloud<PointType>::Ptr input, int mode = 1){
  
  if(mode == 1){//直接用image_projection分割结果
    for(int i=0; i<input->points.size(); i++){
    
      PointType p = input->points[i];
    
      if(p.intensity > segments.size()){
      
        pcl::PointCloud<PointType>::Ptr segment(new pcl::PointCloud<PointType>);
        segment->push_back(p);
        segments.push_back(segment);
      
      }else{
        segments[p.intensity-1]->push_back(p);
      }
    }
  }

  if(mode == 2){//欧几里德聚类分割点云
      pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
      tree->setInputCloud (input);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointType> ec;   
      ec.setClusterTolerance(0.2); 
      ec.setMinClusterSize(15); 
      ec.setMaxClusterSize(25000);
      ec.setSearchMethod(tree); 
      ec.setInputCloud(input);
      ec.extract(cluster_indices);

      for(auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it){  
        pcl::PointCloud<PointType>::Ptr segment(new pcl::PointCloud<PointType>());

        for(auto pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          segment->points.push_back(input->points[*pit]); 
        }

        segments.push_back(segment);
      }
  }

}

/**
 * @name:　describe
 * @msg:　计算点云簇的描述子
 * @param {const} 点云
 * @return {*} 一维向量
 */
Eigen::MatrixXf SegmentFeatures::describe(const pcl::PointCloud<PointType>::Ptr segment){

    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*segment, pca_centroid);

    PclPoint segment_centroid(pca_centroid[0], pca_centroid[1], pca_centroid[2]);
     segmentCentroids.push_back(segment_centroid);

    Eigen::Matrix3f covariance_3d;
    computeCovarianceMatrixNormalized(*segment, pca_centroid, covariance_3d);

    constexpr bool compute_eigenvectors = false;
    Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(covariance_3d, compute_eigenvectors);
    std::vector<float> eigenvalues(3, 0.0);
    eigenvalues.at(0) = eigen_solver.eigenvalues()[0].real();
    eigenvalues.at(1) = eigen_solver.eigenvalues()[1].real();
    eigenvalues.at(2) = eigen_solver.eigenvalues()[2].real();

    // Sort eigenvalues from smallest to largest.
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
    swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

    // Normalize eigenvalues.
    double sum_eigenvalues = eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
    double e1 = eigenvalues.at(0) / sum_eigenvalues;
    double e2 = eigenvalues.at(1) / sum_eigenvalues;
    double e3 = eigenvalues.at(2) / sum_eigenvalues;
    if(e1 == e2 || e2 == e3 || e1 == e3){
      ROS_ERROR("Eigenvalues should not be equal.");
    }

  // Store inside features.
  const double sum_of_eigenvalues = e1 + e2 + e3;
  constexpr double kOneThird = 1.0/3.0;
  // CHECK_NE(e1, 0.0);
  // CHECK_NE(sum_of_eigenvalues, 0.0);

  const double kNormalizationPercentile = 1.0;
  const double kLinearityMax = 2493.5;//28890.9 * kNormalizationPercentile;
  const double kPlanarityMax = 186681.0;//95919.2 * kNormalizationPercentile;
  const double kScatteringMax = 188389.0;//124811 * kNormalizationPercentile;
  const double kOmnivarianceMax = 0.3304;//0.278636 * kNormalizationPercentile;
  const double kAnisotropyMax = 188388.0;//124810 * kNormalizationPercentile;
  const double kEigenEntropyMax = 1.0899;//0.956129 * kNormalizationPercentile;
  const double kChangeOfCurvatureMax = 0.9987;//0.99702 * kNormalizationPercentile;

  const double kNPointsMax = 13200 * kNormalizationPercentile;

  // Feature eigenvalue_feature("eigenvalue");
  // eigenvalue_feature.push_back(FeatureValue("linearity", (e1 - e2) / e1 / kLinearityMax));
  // eigenvalue_feature.push_back(FeatureValue("planarity", (e2 - e3) / e1 / kPlanarityMax));
  // eigenvalue_feature.push_back(FeatureValue("scattering", e3 / e1 / kScatteringMax));
  // eigenvalue_feature.push_back(FeatureValue("omnivariance", std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax));
  // eigenvalue_feature.push_back(FeatureValue("anisotropy", (e1 - e3) / e1 / kAnisotropyMax));
  // eigenvalue_feature.push_back(FeatureValue("eigen_entropy",
  //                                           (e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)) / kEigenEntropyMax));
  // eigenvalue_feature.push_back(FeatureValue("change_of_curvature", e3 / sum_of_eigenvalues / kChangeOfCurvatureMax));
  std::vector<double> eigenvalue_feature;
  eigenvalue_feature.push_back( (e1 - e2) / e1 / kLinearityMax );//linearity
  eigenvalue_feature.push_back( (e2 - e3) / e1 / kPlanarityMax );//planarity
  eigenvalue_feature.push_back( e3 / e1 / kScatteringMax );//scattering
  eigenvalue_feature.push_back( std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax );//omnivariance
  eigenvalue_feature.push_back( (e1 - e3) / e1 / kAnisotropyMax );//anisotropy
  eigenvalue_feature.push_back( (e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)) / kEigenEntropyMax );//eigen_entropy
  eigenvalue_feature.push_back( e3 / sum_of_eigenvalues / kChangeOfCurvatureMax );//change_of_curvature

  PointType point_min, point_max;
  pcl::getMinMax3D(*segment, point_min, point_max);
  
  double diff_x, diff_y, diff_z;

  diff_x = point_max.x - point_min.x;
  diff_y = point_max.y - point_min.y;
  diff_z = point_max.z - point_min.z;

  if (diff_z < diff_x && diff_z < diff_y) {
    // eigenvalue_feature.push_back(FeatureValue("pointing_up", 0.2));
    eigenvalue_feature.push_back( 0.2 );//pointing_up
  } else {
    // eigenvalue_feature.push_back(FeatureValue("pointing_up", 0.0));
    eigenvalue_feature.push_back( 0.0 );//pointing_up
  }

  Eigen::MatrixXf descriptor(1, eigenvalue_feature.size());
  for(int i=0; i<eigenvalue_feature.size(); i++){
    descriptor(0, i) = eigenvalue_feature[i];
  }
  return descriptor;
}

/**
 * @name: computeFeatures
 * @msg: 计算整个frame的特征矩阵
 * @param {*}
 * @return {*}
 */
void SegmentFeatures::computeFeatures(){

  for(auto iter=_segments.begin(); iter!=_segments.end(); ){
    if((*iter)->points.size()<15){
      iter = segments.erase(iter);
    }else{
      iter++;
    }
  }

  featureMatrix.resize(_segments.size(), knn_feature_dim);

  for(int s=0; s<_segments.size(); s++){
      featureMatrix.block<1,knn_feature_dim>(s,0) = 
                            describe(_segments[s]).block<1,knn_feature_dim>(0,0);
  }  
}

/**
 * @name: findMatches
 * @msg: 根据特征描述子，找corresponded segments
 * @param {*}
 * @return {*}
 */  
void SegmentFeatures::findMatches(Eigen::MatrixXf source_matrix, 
                  std::vector<PclPoint> source_centroids, 
                  pcl::CorrespondencesPtr correspondences,
                  pcl::PointCloud<PclPoint>::Ptr first_cloud,
                  pcl::PointCloud<PclPoint>::Ptr second_cloud){
  
  featureMatrix.transposeInPlace();

  int n_nearest_neighbours_limit = std::min(int(featureMatrix.cols()), n_nearest_neighbours);

  nns = Nabo::NNSearchF::createKDTreeLinearHeap(featureMatrix);
  
  int corres_cnt = 0;

  for(int i=0; i<source_matrix.rows(); i++){
    Eigen::VectorXf q = source_matrix.block<1,knn_feature_dim>(i,0).transpose();
    Eigen::VectorXi indices(n_nearest_neighbours_limit);
    Eigen::VectorXf dists2(n_nearest_neighbours_limit);
    
    nns->knn(q, indices, dists2, n_nearest_neighbours_limit);
    
    for (size_t j=0u; j<n_nearest_neighbours_limit; j++){
      if (dists2[j] > feature_distance_threshold){
        break;
      }

      first_cloud->push_back(source_centroids[i]);
      second_cloud->push_back(segmentCentroids[indices[j]]);
      
      PclPoint temp = segmentCentroids[indices[j]];
      double range = std::sqrt(temp.x*temp.x+temp.y*temp.y+temp.z*temp.z);
      if(range < 50){
        correspondences->push_back(pcl::Correspondence(corres_cnt, corres_cnt, 1.0));
      }else{
        correspondences->push_back(pcl::Correspondence(corres_cnt, corres_cnt, 0.2));
      }

      corres_cnt++; 
    }
  }
}

Eigen::MatrixXf SegmentFeatures::getFeatureMatrix(){
  return featureMatrix;
}

std::vector<PclPoint> SegmentFeatures::getSegmentCentroids(){
  return segment_centroids;
}

std::vector<pcl::PointCloud<PointType>::Ptr> SegmentFeatures::getSegments(){
  return segments;
}

};

#endif