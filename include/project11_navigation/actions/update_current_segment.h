#ifndef PROJECT11_NAVIGATION_ACTIONS_UPDATE_CURRENT_SEGMENT_H
#define PROJECT11_NAVIGATION_ACTIONS_UPDATE_CURRENT_SEGMENT_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class UpdateCurrentSegment: public BT::SyncActionNode
{
public:
  UpdateCurrentSegment(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
