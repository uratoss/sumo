#ifndef SUMO_DRIVER_H
#define SUMO_DRIVER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace sumo {
    
class driver {
 public:
  driver();
  void run();

 private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  const double d_;

  void cmd_call_back(const geometry_msgs::Twist::ConstPtr &cmd);
};

} // namespace sumoDriver
#endif
