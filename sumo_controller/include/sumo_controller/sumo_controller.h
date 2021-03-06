#ifndef SUMO_CONTROLLER_H
#define SUMO_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

namespace sumo {
    
class controller {
 public:
  controller();
  void run();

 private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  tf::StampedTransform tf_;

  void ar_call_back(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &marker);
};

} // namespace sumo 
#endif
