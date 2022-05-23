#ifndef SEGMENT_FEATURES_H
#define SEGMENT_FEATURES_H

#include <nabo/nabo.h>
#include "utility.h"

template<typename T>
bool swap_if_gt(T& a, T& b) {
  if (a > b) {
    std::swap(a, b);
    return true;
  }
  return false;
}

class SegmentFeatures{

private:
  std::vector<pcl::PointCloud<PointType>::Ptr> segments;
  
  std::vector<PclPoint> segmentCentroids;
  
  Eigen::MatrixXf featureMatrix;
  
  Nabo::NNSearchF* nns = NULL;
  
  // const int n_nearest_neighbours = 5;
  // const double feature_distance_threshold = 0.6;

public:

  SegmentFeatures(std::vector<pcl::PointCloud<PointType>::Ptr> segments_);
  SegmentFeatures(pcl::PointCloud<PointType>::Ptr input, int mode = 1);

  Eigen::MatrixXf describe(const pcl::PointCloud<PointType>::Ptr segment);

  void computeFeatures();

  void findMatches(Eigen::MatrixXf source_matrix, 
                    std::vector<PclPoint> source_centroids, 
                    pcl::CorrespondencesPtr correspondences,
                    pcl::PointCloud<PclPoint>::Ptr first_cloud,
                    pcl::PointCloud<PclPoint>::Ptr second_cloud);

  Eigen::MatrixXf getFeatureMatrix();

  std::vector<PclPoint> getSegmentCentroids();

  std::vector<pcl::PointCloud<PointType>::Ptr> getSegments();

};

#endif