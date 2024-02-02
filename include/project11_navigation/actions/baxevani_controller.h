#ifndef PROJECT11_NAVIGATION_BAXEVANI_CONTROLLER_H
#define PROJECT11_NAVIGATION_BAXEVANI_CONTROLLER_H

#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace project11_navigation
{

class BaxevaniController: public BT::StatefulActionNode
{
public:
  BaxevaniController(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  nav_msgs::Odometry last_odom_;

  ros::Publisher acceleration_pub_;
  ros::Publisher angular_acceleration_pub_;
  ros::Publisher u_pub_;
  ros::Publisher a_pub_;
  ros::Publisher alpha_pub_;

};

} // namespace path_follower

#endif
