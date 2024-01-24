#ifndef PROJECT11_NAVIGATION_ACTIONS_VISUALIZE_TRAJECTORY_H
#define PROJECT11_NAVIGATION_ACTIONS_VISUALIZE_TRAJECTORY_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class VisualizeTrajectory: public BT::SyncActionNode
{
public:
  VisualizeTrajectory(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
