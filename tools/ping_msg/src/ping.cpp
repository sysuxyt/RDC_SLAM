#include <ros/ros.h>
#include "ping_msg/Ping.h"
#include <iostream>


double parsePingResult(std::string s){
    int pos = s.find("time"); //改成transmitted
    int found = 0; 
    float ms; 
    if(pos != std::string::npos){
        found = sscanf(s.substr(pos).c_str(),"time %f",&ms);//transmitted， %f 
    }
    if(found == 1 && pos != std::string::npos) {
        return ms; 
    }
    else{
        return 10000;
    }
}

int main(int argc, char **argv)
{

    ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");


  ros::init(argc, argv, "ping_msg");

  ros::NodeHandle nh;
  
  ros::Publisher _pubPing = nh.advertise<ping_msg::Ping>("ping_msgs", 1);

  double p_time=25; 
  char line[200]; 
  FILE *p;

  ros::Rate rate(2.0);

  while(ros::ok()){
      ros::spinOnce();

      p = popen("ping -c 1 14.215.177.39","r");//换
      fgets(line, 200, p);
      fgets(line, 200, p); 
      fgets(line, 200, p); 
      fgets(line, 200, p); 
      fgets(line, 200, p); 
    //   std::cout<<line<<std::endl;
      pclose(p);

      double res_time = parsePingResult(line);
      std::cout<<"-----time: "<<res_time<<std::endl;
      if(res_time < 100){
        ping_msg::Ping rosmsg;

        rosmsg.header.stamp = ros::Time::now();

        rosmsg.robotFrom = 0;//换 
        rosmsg.robotTo   = 1;//换

        _pubPing.publish(rosmsg);
        // ROS_INFO("receive!");
      }
      rate.sleep();
  }

  return 0;

}
