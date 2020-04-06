#include "sumo_driver/sumo_driver.h"
#include "sumo_driver/Zop.h"

namespace sumo {

driver::driver() : d_(0.05) {
  sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10,
                                           &driver::cmd_call_back, this);
  pub = nh.advertise<sumo_driver::Zop>("operation", 5);
}

void driver::run() { ros::spin(); }

void driver::cmd_call_back(const geometry_msgs::Twist::ConstPtr &cmd) {
  sumo_driver::Zop zumo_operation;
  // ホロノミックな目標速度から非ホロノミックな目標速度に変換
  const double cmd_right = (cmd->linear.x + d_ * cmd->angular.z);
  const double cmd_left = (cmd->linear.x - d_ * cmd->angular.z);
  // 目標速度から操作量に変換
  zumo_operation.right = cmd_right * 200;
  zumo_operation.left = cmd_left * 200;

  zumo_operation.left = (std::abs(zumo_operation.left) > 400)
                            ? std::copysign(400, zumo_operation.left)
                            : zumo_operation.left;
  zumo_operation.right = (std::abs(zumo_operation.right) > 400)
                             ? std::copysign(400, zumo_operation.right)
                             : zumo_operation.right;
  pub.publish(zumo_operation);

  ROS_INFO_STREAM("left:" << zumo_operation.left
                          << " right:" << zumo_operation.right);
}

}  // namespace sumo

int main(int argc, char **argv) {
  ros::init(argc, argv, "sumo_driver");

  sumo::driver driver;

  driver.run();

  return 0;
}
