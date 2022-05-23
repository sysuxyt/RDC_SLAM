#include "msgFactory.h"

/**
 * @name: 
 * @msg: RobotMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* RobotMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  if (skipHeader)
    return c;
  c = _toCharArray(type(), c, bsize);
  c = _toCharArray(_robotId, c, bsize);
  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组RobotMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* RobotMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  if (skipHeader)
    return c;
  int t;
  c = _fromCharArray(t, c);
  c = _fromCharArray(_robotId, c);
  assert (t==type() && "FATAL, TYPE MISMATCH");
  return c;
}

/**
 * @name: 
 * @msg: VertexArrayMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* VertexArrayMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s =vertexVector.size();
  c = _toCharArray<size_t>(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< vertexVector.size(); i++){
    c = _toCharArray(vertexVector[i].id, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[0], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[1], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[2], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[3], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[4], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[5], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组VertexArrayMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* VertexArrayMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  size_t s;
  c = _fromCharArray(s,c);
  vertexVector.resize(s);
  for (size_t i =0; i< vertexVector.size(); i++){
    c = _fromCharArray(vertexVector[i].id, c);
    c = _fromCharArray(vertexVector[i].estimate[0], c);
    c = _fromCharArray(vertexVector[i].estimate[1], c);
    c = _fromCharArray(vertexVector[i].estimate[2], c);
    c = _fromCharArray(vertexVector[i].estimate[3], c);
    c = _fromCharArray(vertexVector[i].estimate[4], c);
    c = _fromCharArray(vertexVector[i].estimate[5], c);
  }
  return c;
}

/**
 * @name: 
 * @msg: RobotLaserMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* RobotLaserMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer; 
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  c = _toCharArray(nodeId, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s = laserCloudCorner->points.size();
  c = _toCharArray(s, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  s = laserCloudSurf->points.size();
  c = _toCharArray(s, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  s = laserCloudOutlier->points.size();
  c = _toCharArray(s, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< laserCloudCorner->points.size(); i++){
    c = _toCharArray(laserCloudCorner->points[i].x, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudCorner->points[i].y, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudCorner->points[i].z, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudCorner->points[i].intensity, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }
  for (size_t i =0; i< laserCloudSurf->points.size(); i++){
    c = _toCharArray(laserCloudSurf->points[i].x, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudSurf->points[i].y, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudSurf->points[i].z, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudSurf->points[i].intensity, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }
  for (size_t i =0; i< laserCloudOutlier->points.size(); i++){
    c = _toCharArray(laserCloudOutlier->points[i].x, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudOutlier->points[i].y, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudOutlier->points[i].z, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(laserCloudOutlier->points[i].intensity, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组RobotLaserMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* RobotLaserMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = _fromCharArray(nodeId,c);
  
  size_t s,s1,s2;
  c = _fromCharArray(s,c);
  laserCloudCorner.reset(new pcl::PointCloud<PointType>());
  c = _fromCharArray(s1,c);
  laserCloudSurf.reset(new pcl::PointCloud<PointType>());
  c = _fromCharArray(s2,c);
  laserCloudOutlier.reset(new pcl::PointCloud<PointType>());

  for (size_t i =0; i< s; i++){
    PointType point;
    c = _fromCharArray(point.x, c);
    c = _fromCharArray(point.y, c);
    c = _fromCharArray(point.z, c);
    c = _fromCharArray(point.intensity, c);
    laserCloudCorner->push_back(point);
  }
  for (size_t i =0; i< s1; i++){
    PointType point;
    c = _fromCharArray(point.x, c);
    c = _fromCharArray(point.y, c);
    c = _fromCharArray(point.z, c);
    c = _fromCharArray(point.intensity, c);
    laserCloudSurf->push_back(point);
  }
  for (size_t i =0; i< s2; i++){
    PointType point;
    c = _fromCharArray(point.x, c);
    c = _fromCharArray(point.y, c);
    c = _fromCharArray(point.z, c);
    c = _fromCharArray(point.intensity, c);
    laserCloudOutlier->push_back(point);
  }

  return c;
}

/**
 * @name: 
 * @msg: MapMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* MapMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer; 
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  c = _toCharArray(inclosure, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  c = _toCharArray(outclosure, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s = map->points.size();
  c = _toCharArray(s, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< map->points.size(); i++){
    c = _toCharArray(map->points[i].x, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(map->points[i].y, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(map->points[i].z, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(map->points[i].intensity, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组MapMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* MapMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = _fromCharArray(inclosure,c);
  c = _fromCharArray(outclosure,c);
  
  size_t s;
  c = _fromCharArray(s,c);
  map.reset(new pcl::PointCloud<PointType>());

  for (size_t i =0; i< s; i++){
    PointType point;
    c = _fromCharArray(point.x, c);
    c = _fromCharArray(point.y, c);
    c = _fromCharArray(point.z, c);
    c = _fromCharArray(point.intensity, c);
    map->push_back(point);
  }

  return c;
}

// char* ComboMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
//   char* c = buffer;
//   size_t free = bsize;
//   c = RobotMessage::toCharArray(c, bsize, skipHeader);
//   free = (c) ? bsize - (c - buffer) : 0;
//   c = VertexArrayMessage::toCharArray(c, free, true);
//   free = (c) ? bsize - (c - buffer) : 0;
//   c = RobotLaserMessage::toCharArray(c, free, true);
//   return c;
// }

// const char* ComboMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
//   const char* c = buffer;
//   c = RobotMessage::fromCharArray(c, bsize, skipHeader);
//   c = VertexArrayMessage::fromCharArray(c, bsize, true);
//   c = RobotLaserMessage::fromCharArray(c, bsize, true);
//   return c;
// }

/**
 * @name: 
 * @msg: ComboMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* ComboMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  c = _toCharArray(nodeId, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  int s = descriptor.cols();
  c = _toCharArray(s, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (int i =0; i< s; i++){
    c = _toCharArray(descriptor(0,i), c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组ComboMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* ComboMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = _fromCharArray(nodeId,c);
  
  int s;
  c = _fromCharArray(s,c);
  descriptor.resize(1,s);

  for (int i =0; i< s; i++){
    c = _fromCharArray(descriptor(0,i), c);
  }

  return c;
}

/**
 * @name: 
 * @msg: RPMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* RPMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  //含match candidates数量
  size_t s = candidates.size();
  c = _toCharArray(s, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< s; i++){
    //本match_candidate对应的frame id
    c = _toCharArray(candidateIdPairs[i].first, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(candidateIdPairs[i].second, c, free);
    free = (c) ? bsize - (c - buffer) : 0;

    //本match_candidate中segments数量
    size_t row = candidateCentroids[i].size();
    c = _toCharArray(row, c, free);
    free = (c) ? bsize - (c - buffer) : 0;

    for(size_t j=0; j<row; j++){//单个segment的元素
      //segment的中心点
      c = _toCharArray(candidateCentroids[i][j].x, c, free);
      free = (c) ? bsize - (c - buffer) : 0;
      c = _toCharArray(candidateCentroids[i][j].y, c, free);
      free = (c) ? bsize - (c - buffer) : 0;
      c = _toCharArray(candidateCentroids[i][j].z, c, free);
      free = (c) ? bsize - (c - buffer) : 0;

      //segment对应的描述子
      for(int k=0; k<knn_feature_dim; k++){
        c = _toCharArray(candidates[i](j,k), c, free);
        free = (c) ? bsize - (c - buffer) : 0;
      }
    }
  }
  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组RPMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* RPMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  
  //含match candidates数量
  size_t s;
  c = _fromCharArray(s,c);
  // 分配内存  
  candidates.resize(s);
  candidateIdPairs.resize(s);
  candidateCentroids.resize(s);

  for (size_t i=0; i<s; i++){//单个match candidate
    
    //本match_candidate对应的frame id存入candidateIdPairs
    int candidate_id1, candidate_id2;
    c = _fromCharArray(candidate_id1, c);
    c = _fromCharArray(candidate_id2, c);
    std::pair<int,int> candidateIdPair(candidate_id1, candidate_id2);
    candidateIdPairs[i] = candidateIdPair;

    //本match_candidate中segments数量
    size_t row;
    c = _fromCharArray(row,c);
    //给第i个segment对应数据分配内存  
    candidates[i].resize(row, knn_feature_dim);
    candidateCentroids[i].resize(row);

    for(size_t j=0; j<row; j++){//单个segment的元素
      //segment的中心点  第i个candidate，第j个segment
      c = _fromCharArray(candidateCentroids[i][j].x, c);
      c = _fromCharArray(candidateCentroids[i][j].y, c);
      c = _fromCharArray(candidateCentroids[i][j].z, c);

      //segment对应的描述子 第i个candidate，第j个segment
      for(int k=0; k<knn_feature_dim; k++){
        c = _fromCharArray(candidates[i](j,k), c);
      }

    }

  }
  return c;
}

/**
 * @name: 
 * @msg: EdgeArrayMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* EdgeArrayMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s =edgeVector.size();
  c = _toCharArray<size_t>(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< edgeVector.size(); i++){
    c = _toCharArray(edgeVector[i].idfrom, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].idto, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].inter, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[0], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[1], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[2], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[3], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[4], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[5], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    for(int j=0;j<36;j++){
      c = _toCharArray(edgeVector[i].information[j], c, free);
      free = (c) ? bsize - (c - buffer) : 0;
    }
  }

  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组EdgeArrayMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* EdgeArrayMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  size_t s;
  c = _fromCharArray(s,c);
  edgeVector.resize(s);
  for (size_t i =0; i< edgeVector.size(); i++){
    c = _fromCharArray(edgeVector[i].idfrom, c);
    c = _fromCharArray(edgeVector[i].idto, c);
    c = _fromCharArray(edgeVector[i].inter, c);
    c = _fromCharArray(edgeVector[i].estimate[0], c);
    c = _fromCharArray(edgeVector[i].estimate[1], c);
    c = _fromCharArray(edgeVector[i].estimate[2], c);
    c = _fromCharArray(edgeVector[i].estimate[3], c);
    c = _fromCharArray(edgeVector[i].estimate[4], c);
    c = _fromCharArray(edgeVector[i].estimate[5], c);
    for(int j=0;j<36;j++){
      c = _fromCharArray(edgeVector[i].information[j], c);
    }
  }
  return c;
}

// // serializes the message in the buffer
// char* ClosuresMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
//   char* c = buffer;
//   size_t free = bsize;

//   c = RobotMessage::toCharArray(c, free, skipHeader);
//   free = (c) ? bsize - (c - buffer) : 0;

//   size_t s =closures.size();
//   c = _toCharArray<size_t>(s,c, free);
//   free = (c) ? bsize - (c - buffer) : 0;

//   for (size_t i =0; i< closures.size(); i++){
//     c = _toCharArray(closures[i], c, free);
//     free = (c) ? bsize - (c - buffer) : 0;
//   }

//   return c;
// }

// const char* ClosuresMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
//   const char* c = buffer;
//   c = RobotMessage::fromCharArray(c, bsize, skipHeader);
//   size_t s;
//   c = _fromCharArray(s,c);
//   closures.resize(s);
//   for (size_t i =0; i< closures.size(); i++)
//     c = _fromCharArray(closures[i], c);
  
//   return c;
// }

// char* CondensedGraphMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
//   char* c = buffer;
//   c = RobotMessage::toCharArray(c, bsize, skipHeader);
//   c = EdgeArrayMessage::toCharArray(c, bsize, true);
//   c = ClosuresMessage::toCharArray(c, bsize, true);
//   return c;
// }

// const char* CondensedGraphMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
//   const char* c = buffer;
//   c = RobotMessage::fromCharArray(c, bsize, skipHeader);
//   c = EdgeArrayMessage::fromCharArray(c, bsize, true);
//   c = ClosuresMessage::fromCharArray(c, bsize, true);
//   return c;
// }

/**
 * @name: 
 * @msg: DistGraphMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* DistGraphMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;
  c = RobotMessage::toCharArray(c, bsize, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  c = _toCharArray(isInit, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  c = _toCharArray(hasEdge, c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s = linearizeRotations.size();
  c = _toCharArray<size_t>(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (auto& key_value : linearizeRotations){
    gtsam::Key key = key_value.first;
    int index = gtsam::symbolIndex(key);

    c = _toCharArray(index, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    
    c = _toCharArray(linearizeRotations.at(key)[0], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[1], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[2], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[3], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[4], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[5], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[6], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[7], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizeRotations.at(key)[8], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  s =linearizePoses.size();
  c = _toCharArray<size_t>(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (auto& key_value : linearizePoses){
    gtsam::Key key = key_value.first;
    int index = gtsam::symbolIndex(key);

    c = _toCharArray(index, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    
    c = _toCharArray(linearizePoses.at(key)[0], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizePoses.at(key)[1], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizePoses.at(key)[2], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizePoses.at(key)[3], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizePoses.at(key)[4], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(linearizePoses.at(key)[5], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  c = EdgeArrayMessage::toCharArray(c, free, true);
  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组DistGraphMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* DistGraphMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);

  c = _fromCharArray(isInit, c);
  c = _fromCharArray(hasEdge, c);


  size_t s;
  c = _fromCharArray(s,c);
  for (size_t i =0; i< s; i++){
    gtsam::Vector value=gtsam::Vector::Zero(9);
    int index;

    c = _fromCharArray(index, c);
    c = _fromCharArray(value[0], c);
    c = _fromCharArray(value[1], c);
    c = _fromCharArray(value[2], c);
    c = _fromCharArray(value[3], c);
    c = _fromCharArray(value[4], c);
    c = _fromCharArray(value[5], c);
    c = _fromCharArray(value[6], c);
    c = _fromCharArray(value[7], c);
    c = _fromCharArray(value[8], c);

    linearizeRotations.insert(gtsam::Symbol(ROBOT_NAMES[_robotId],index), value);
  }

  c = _fromCharArray(s,c);
  for (size_t i =0; i< s; i++){
    gtsam::Vector value=gtsam::Vector::Zero(6);
    int index;

    c = _fromCharArray(index, c);
    c = _fromCharArray(value[0], c);
    c = _fromCharArray(value[1], c);
    c = _fromCharArray(value[2], c);
    c = _fromCharArray(value[3], c);
    c = _fromCharArray(value[4], c);
    c = _fromCharArray(value[5], c);

    linearizePoses.insert(gtsam::Symbol(ROBOT_NAMES[_robotId],index), value);
  }

  c = EdgeArrayMessage::fromCharArray(c, bsize, true);
  return c;
}

/**
 * @name: 
 * @msg: GraphMessage消息序列化
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
char* GraphMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;
  c = RobotMessage::toCharArray(c, bsize, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;
  c = _toCharArray(targetVertex, c, free);
  free = (c) ? bsize - (c - buffer) : 0;
  c = VertexArrayMessage::toCharArray(c, free, true);
  free = (c) ? bsize - (c - buffer) : 0;
  c = EdgeArrayMessage::toCharArray(c, free, true);
  return c;
}

/**
 * @name: 
 * @msg: 从序列化数据中重组GraphMessage消息
 * @param {const} char
 * @param {size_t} bsize
 * @param {bool} skipHeader
 * @return {*}
 */
const char* GraphMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = _fromCharArray(targetVertex, c);
  c = VertexArrayMessage::fromCharArray(c, bsize, true);
  c = EdgeArrayMessage::fromCharArray(c, bsize, true);
  return c;
}

/**
 * @name: 
 * @msg: 
 * @param {int} t
 * @return {*}
 */
RobotMessage* MessageFactory::constructMessage(int t) {
  std::map<int,BaseMessageCreator*>::iterator it = _msgCreatorMap.find(t);
  if (it==_msgCreatorMap.end())
    return 0;
  return (it->second)->constructMessage();
}

/**
 * @name: 
 * @msg: 
 * @param {const} char
 * @param {size_t} size
 * @return {*}
 */
RobotMessage* MessageFactory::fromCharArray(const char* buf, size_t size) {
  assert (size > sizeof(int));
  int type;
  _fromCharArray(type, buf);
  const char* c = buf;
  RobotMessage* msg = constructMessage(type);
  cerr << "constructing a message of type" << type << endl;
  c = msg->fromCharArray(c);
  assert((c-buf)==size);
  return msg;
}
  

