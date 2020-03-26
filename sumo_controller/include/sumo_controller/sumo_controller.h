#ifndef SUMO_CONTROLLER_H
#define SUMO_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
  ros::Time prev_time;
  Eigen::Vector3d prev_position;
  bool is_first_;

  void ar_call_back(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &marker);
};

} // namespace sumo 
#endif
