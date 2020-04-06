#include <geometry_msgs/Twist.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "sumo_controller/sumo_controller.h"

namespace sumo {
controller::controller() {
  sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>(
      "/ar_pose_marker", 10, &controller::ar_call_back, this);
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);

  tf::TransformListener listener;
  try {
    listener.waitForTransform("/base_link", "/camera", ros::Time(0),
                              ros::Duration(10.0));
    listener.lookupTransform("/base_link", "/camera", ros::Time(0), tf_);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
}

void controller::run() { ros::spin(); }

void controller::ar_call_back(
    const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &markers) {
  geometry_msgs::Twist cmd;

  if (markers->markers.empty()) {
    // cmd.linear.x = 0.0;
    // cmd.angular.z = 6 * boost::math::constants::pi<double>();
    // cmd.angular.z = 0.0;
    pub.publish(cmd);
    return;
  }

  ar_track_alvar_msgs::AlvarMarker marker = markers->markers[0];

  tf::Vector3 position_or = tf_ * tf::Vector3(marker.pose.pose.position.x,
                                              marker.pose.pose.position.y,
                                              marker.pose.pose.position.z);
  const double &x = position_or.x();
  const double &y = position_or.y();
  const double &z = position_or.z();

  const double th = std::atan2(y, x);
  const double hypot = std::hypot(x, y);

  ROS_INFO_STREAM("hypot : " << hypot << " th : " << th);
  if (std::abs(th) > boost::math::constants::pi<double>() / 32) {
    cmd.angular.z = std::copysign(20, th);
  } else if (hypot > 0.15) {
    // cmd.linear.x = hypot / 2;
    cmd.linear.x = 1;
  }

  ROS_INFO_STREAM("x : " << cmd.linear.x << " z : " << cmd.angular.z);

  pub.publish(cmd);
}

// void controller::joy_call_back(const sensor_msgs::Joy::ConstPtr &joy) {
//   geometry_msgs::Twist cmd;
//   cmd.linear.x = joy->axes[1];
//   cmd.angular.z = joy->axes[0];
//   ROS_INFO_STREAM("x " << cmd.linear.x << " z " << cmd.angular.z);
//   pub.publish(cmd);
// }

}  // namespace sumo

int main(int argc, char **argv) {
  ros::init(argc, argv, "sumo_controller");

  try {
    sumo::controller controller;
    controller.run();
  } catch (tf::TransformException &ex) {
    return -1;
  }

  return 0;
}
