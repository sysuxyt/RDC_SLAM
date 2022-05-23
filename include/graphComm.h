#ifndef _GRAPH_COMM_H_
#define _GRAPH_COMM_H_

#include <sys/socket.h> /* socket specific definitions */
#include <arpa/inet.h> /* IP address conversion stuff */

#include <string>
#include <queue>

#include <ros/ros.h>
#include "mapOptimization.h"
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>

#define SIM_EXPERIMENT 0
#define REAL_EXPERIMENT 1
#define BAG_EXPERIMENT 2

// #define SIM_COMM_RANGE 15.0
#define COMM_TIME 10.0

typedef std::queue<StampedRobotMessage> msgQueue;

enum TypeExperiment {SIM, REAL, BAG};

class GraphComm{
 public:
  GraphComm(mapOptimization* gslam, int idRobot, int nRobots, std::string base_addr, TypeExperiment typeExperiment);

  // void init_network(SE2 gtPoses[]);
  // void init_network(struct timeval pings[]);
  void init_network(ros::NodeHandle* rh);

 protected:
  bool inCommunicationRange(int r1, int r2);
  bool robotsInRange(std::vector<int>& robotsToSend);
  int send(RobotMessage* cmsg, int rto);
  void sendToThrd();
  RobotMessage* receive();
  void receiveFromThrd();
  void processQueueThrd();
  void init_threads();
  void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, Eigen::Vector3d *gtpose);
  // void groundTruthCallback(const sensor_msgs::NavSatFix::ConstPtr& msg, Eigen::Vector3d *gtpose);
  Eigen::Vector3d NavSatFix2UTM(const sensor_msgs::NavSatFix::ConstPtr& msg);

  mapOptimization* _gslam;

  int _iSock;
  int _idRobot;
  int _nRobots;
  string _rootns;
  TypeExperiment _typeExperiment;

  std::string _baseAddr;

  msgQueue _queue;
  boost::mutex _queueMutex;

  ros::NodeHandle *_rh;
  ros::Subscriber *_subgt;
  Eigen::Vector3d *_gtPoses;

  boost::thread sthread;
  boost::thread rthread;
  boost::thread pthread;

  // std::ofstream transmitBytesOfs;
  // std::ofstream timeOfModulesOfs;
};


#endif
