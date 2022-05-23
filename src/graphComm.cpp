#include "graphComm.h"


GraphComm::GraphComm (mapOptimization* gslam, int idRobot, int nRobots, std::string base_addr, TypeExperiment typeExperiment){
  _typeExperiment = typeExperiment;

  _idRobot = idRobot;
  _nRobots = nRobots;

  _rootns = "/robot";

  _base_addr = base_addr;

  _gslam = gslam;

  _gtPoses = new Eigen::Vector3d[nRobots];
  _subgt = new ros::Subscriber[nRobots];

  std::stringstream my_addr;
  my_addr << base_addr << idRobot+1;//===============my_addr
  // my_addr << base_addr << 1;//===============my_addr
  std::cerr << "My address: " << my_addr.str() << std::endl;

  _iSock = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

  struct sockaddr_in sockAddr;
  sockAddr.sin_family=AF_INET;
  sockAddr.sin_addr.s_addr=inet_addr(my_addr.str().c_str());
  sockAddr.sin_port=htons(42001);//12.02 ？？端口号是不是要改下，收发端口号不同
  bind(_iSock,(struct sockaddr*)&sockAddr,sizeof(sockAddr));

  if(_typeExperiment == SIM){
    for (int r = 0; r < _nRobots; r++){
      std::stringstream nametopic;
      nametopic << _rootns << r << "/base_pose_ground_truth";
      geometry_msgs::PoseStamped::ConstPtr gtmsg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(nametopic.str());
      _gtPoses[r] = Eigen::Vector3d(gtmsg->pose.position.x, gtmsg->pose.position.y, gtmsg->pose.position.z);
      // sensor_msgs::NavSatFix::ConstPtr gtmsg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(nametopic.str());//navsatfix
      // _gtPoses[r] = NavSatFix2UTM(gtmsg);//navsatfix
    }
  }

  // transmitBytesOfs.open("/home/xyt/cslam_final/tmp/bytes_record.txt");
  // transmitBytesOfs<<"timestamp pr_bytes rp_bytes dgo_bytes"<<std::endl;
  // timeOfModulesOfs.open("/home/xyt/cslam_final/tmp/time_record.txt");
  // timeOfModulesOfs<<"timestamp pr_time rp_time dgo_time"<<std::endl;
}

void GraphComm::groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, Eigen::Vector3d *gtpose)//poseStamped
{ 
  *gtpose = Eigen::Vector3d(msg->pose.position.x , msg->pose.position.y , msg->pose.position.z);
}

Eigen::Vector3d GraphComm::NavSatFix2UTM(const sensor_msgs::NavSatFix::ConstPtr& navsat_msg){
  geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
  gps_msg->position.latitude = navsat_msg->latitude;
  gps_msg->position.longitude = navsat_msg->longitude;
  gps_msg->position.altitude = navsat_msg->altitude;
  geodesy::UTMPoint utm;
  geodesy::fromMsg(gps_msg->position, utm);
  return Eigen::Vector3d(utm.easting , utm.northing , utm.altitude);
}
// void GraphComm::groundTruthCallback(const sensor_msgs::NavSatFix::ConstPtr& msg, Eigen::Vector3d *gtpose)//navsatfix
// { 
//   *gtpose = NavSatFix2UTM(msg);
// }

void GraphComm::init_threads(){
  sthread = boost::thread(&GraphComm::sendToThrd, this);
  rthread = boost::thread(&GraphComm::receiveFromThrd, this);
  pthread = boost::thread(&GraphComm::processQueueThrd, this);
}

void GraphComm::init_network(ros::NodeHandle* rh){
  _rh = rh;

  if(_typeExperiment == SIM){
    for (int r = 0; r < _nRobots; r++){
      std::stringstream nametopic;
      nametopic << _rootns << r << "/base_pose_ground_truth";
      _subgt[r] = _rh->subscribe<geometry_msgs::PoseStamped>(nametopic.str(), 1, boost::bind(&GraphComm::groundTruthCallback, this, _1, &_gtPoses[r]));
      // _subgt[r] = _rh->subscribe<sensor_msgs::NavSatFix>(nametopic.str(), 1, boost::bind(&GraphComm::groundTruthCallback, this, _1, &_gtPoses[r]));//navsatfix

    }
  }

  init_threads();
}

bool GraphComm::inCommunicationRange(int r1, int r2){
  //  std::cout<<_gtPoses[r1]<<"-------"<<std::endl;
  // std::cout<<_gtPoses[r2]<<"-------"<<std::endl;
  // std::cout<<_gtPoses[r1]-_gtPoses[r2]<<"-------"<<std::endl;
  // std::cout<<"*****"<<std::sqrt((_gtPoses[r1] - _gtPoses[r2]).dot(_gtPoses[r1] - _gtPoses[r2]))<<std::endl;
  return (_gtPoses[r1] - _gtPoses[r2]).dot(_gtPoses[r1] - _gtPoses[r2]) < SIM_COMM_RANGE * SIM_COMM_RANGE;
}

bool GraphComm::robotsInRange(std::vector<int>& robotsToSend){
  robotsToSend.clear();
  //if(_idRobot==0) return false;

  if (_typeExperiment == REAL){
    //Send to all... the message will arrive if they are in range
    for (int r = 0; r < _nRobots; r++){
      if (r != _idRobot) //Except to me!
	      robotsToSend.push_back(r);
    }
  }else if (_typeExperiment == SIM){
    //Send if inCommunicationRange
    for (int r = 0; r < _nRobots; r++){
      if (r != _idRobot){
	      //Looking for ground truth pose
        if (inCommunicationRange(_idRobot, r)){
          robotsToSend.push_back(r);
          // ROS_INFO("in communication range!!!");
        }
      }
    }
  }
  // else if (_typeExperiment == BAG){
  //   //Send if recent ping
  //   ros::Time curr_time = ros::Time::now();
  //   for (int r = 0; r < _nRobots; r++){
  //     if (r != _idRobot){
  //       if ((curr_time.toSec() -_rh->getTimeLastPing(r).toSec()) < COMM_TIME){ //Less than COMM_TIME seconds since last ping
  //         robotsToSend.push_back(r);
  //       }
  //     }
  //   }
  // }

  return robotsToSend.size();
}

int GraphComm::send(RobotMessage* cmsg, int rto){
  std::stringstream to_addr;
  to_addr << _base_addr << rto +1;//===================to_add
  // to_addr << _base_addr << 1;//===================to_add

  struct sockaddr_in toSockAddr;
  toSockAddr.sin_family=AF_INET;
  toSockAddr.sin_addr.s_addr=inet_addr(to_addr.str().c_str());
  toSockAddr.sin_port=htons(42001);
  
  char bufferc [MAX_LENGTH_MSG];
  char* c = cmsg->toCharArray(bufferc, MAX_LENGTH_MSG); //  create buffer;
  size_t sizebufc = (c) ? (c-bufferc):0;

  if (sizebufc){
    std::cerr << "Send info to robot: " << rto << ". Address: " << to_addr.str() << ". Sent: " << sizebufc  << " bytes" << std::endl;
    sendto(_iSock, &bufferc, sizebufc, 0, (struct sockaddr*) &toSockAddr, sizeof(toSockAddr));
    //if (_typeExperiment == REAL)
     // _rh->publishSentMsg(cmsg);
  }
  return sizebufc;
}



void GraphComm::sendToThrd(){

  static int transmit_bytes_pr = 0;
  static int transmit_bytes_rp = 0;
  static int transmit_bytes_dgo = 0;


  int lastSentVertex = -1;
  std::vector<int> robotsToSend;
  while(1){
    if (robotsInRange(robotsToSend)){
      if (_gslam->lastVertex() > lastSentVertex+1){
        
        lastSentVertex = _gslam->lastVertex();

        bool flag = false;
        for (size_t i = 0; i < robotsToSend.size(); i++){
          if(_gslam->lastVertex() > _gslam->lastSepNode(robotsToSend[i])+SEPNODE_STEP){
            flag = true;
            // robotsToSend.erase(robotsToSend.begin()+i);
            // cout<<"robotstosend num:"<<robotsToSend.size()<<std::endl;
          }
        }

        if(flag){
          //RobotLaserMessage* rmsg = _gslam->constructRobotLaserMessage();
          ComboMessage* cmsg = _gslam->constructComboMessage();
          if(cmsg != NULL){
            //Send to robots in range
            for (size_t i = 0; i < robotsToSend.size(); i++){
              int rto = robotsToSend[i];
              std::cerr << "Send PR msg:";
              transmit_bytes_pr += send(cmsg, rto);
            }
          }
        }

      }

      while(!_gslam->outRPMsg.empty()){
        RPMessage* temp = _gslam->outRPMsg.front();
        //if(robotsToSend.find(temp.first)){
        std::cerr << "Send RP msg:";
        
        for (size_t i = 0; i < robotsToSend.size(); i++){
          int rto = robotsToSend[i];
          transmit_bytes_rp += send(temp, rto);
        }

        _gslam->outRPMsg.pop_front();
      
      }

      // while(!_gslam->outDistGraphMsg.empty()){
      //   DistGraphMessage* temp = _gslam->outDistGraphMsg.front();
      //   //if(robotsToSend.find(temp.first)){
      //   std::cerr << "Send DGO msg:";
        
      //   for (size_t i = 0; i < robotsToSend.size(); i++){
      //     int rto = robotsToSend[i];
      //     transmit_bytes_dgo += send(temp, rto);
      //   }

      //   _gslam->outDistGraphMsg.pop_front();
      
      // }
      if(send_map){//12.02
        while(!_gslam->outMapMsg.empty()){
          pair<int, MapMessage*> temp = _gslam->outMapMsg.front();
          //if(robotsToSend.find(temp.first)){
            // std::cerr << "Send map msg:";
            // send(temp.second, temp.first);
          //}
          if(temp.second != NULL){
            //Send to robots in range
            for (size_t i = 0; i < robotsToSend.size(); i++){
              int rto = robotsToSend[i];
              std::cerr << "Send MAP:";
              send(temp.second, rto);
            }
          }
          _gslam->outMapMsg.pop_front();
        }
      }

    }
    while(!_gslam->outDistGraphMsg.empty()){
      DistGraphMessage* temp = _gslam->outDistGraphMsg.front();
      //if(robotsToSend.find(temp.first)){
      std::cerr << "Send DGO msg:";
      
      // for (size_t i = 0; i < robotsToSend.size(); i++){
      for (int rto = 0; rto < _nRobots; rto++){ 
        // int rto = robotsToSend[i];
        if(rto!=_idRobot){
          transmit_bytes_dgo += send(temp, rto);
        }
      }

      _gslam->outDistGraphMsg.pop_front();
    
    }

    // transmitBytesOfs<<ros::Time::now()<<" "
    //                   <<transmit_bytes_pr<<" "
    //                   <<transmit_bytes_rp<<" "
    //                   <<transmit_bytes_dgo<<std::endl;

    // timeOfModulesOfs<<ros::Time::now()<<" "
    //                   <<_gslam->get_time_pr()<<" "
    //                   <<_gslam->get_time_rp()<<" "
    //                   <<_gslam->get_time_dgo()<<std::endl;

    usleep(50000);
  }
}

RobotMessage* GraphComm::receive(){
  struct sockaddr_in toSockAddr;
  int toSockAddrLen=sizeof(struct sockaddr_in);
  
    // std::cerr<<MAX_LENGTH_MSG<<std::endl;

  int sizebuf = MAX_LENGTH_MSG;
  char buffer[sizebuf];

  int nbytes = recvfrom(_iSock, &buffer, sizebuf ,0,(struct sockaddr*)&toSockAddr, (socklen_t*)&toSockAddrLen);
  fprintf(stderr, "Received %i bytes.\n", nbytes);

  //////////////////
  //Deserialize data
  RobotMessage *msg = _gslam->createMsgfromCharArray(buffer, nbytes);

  return msg;
}

void GraphComm::receiveFromThrd(){

  while(1){
    //////////////////
    //Receive data
    RobotMessage* msg = receive();

    fprintf(stderr, "Received info from: %i\n", msg->robotId());
    if (_typeExperiment == REAL){
      //_rh->publishReceivedMsg(msg);
      //_rh->publishPing(msg->robotId());
    }

    StampedRobotMessage smsg;
    smsg.msg = msg;
    smsg.refVertex = _gslam->lastVertex();

    boost::mutex::scoped_lock lock(_queueMutex);
    _queue.push(smsg);
  }
}

void GraphComm::processQueueThrd(){

  while(1){
    if (!_queue.empty()){
      boost::mutex::scoped_lock lock(_queueMutex);

      StampedRobotMessage smsg = _queue.front();
      _gslam->addInterRobotData(smsg);

      _queue.pop();
    }else
      usleep(80000);
  }
}





