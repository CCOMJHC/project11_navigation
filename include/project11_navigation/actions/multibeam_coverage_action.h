#ifndef PROJECT11_NAVIGATION_ACTIONS_MULTIBEAM_COVERAGE_ACTION_H
#define PROJECT11_NAVIGATION_ACTIONS_MULTIBEAM_COVERAGE_ACTION_H

#include <behaviortree_cpp/bt_factory.h>
#include <actionlib/client/simple_action_client.h>
#include <project11_nav_msgs/multibeam_coverageAction.h>

namespace project11_navigation
{

class MultibeamCoverageAction: public BT::StatefulActionNode
{
public:
  MultibeamCoverageAction(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<actionlib::SimpleActionClient<project11_nav_msgs::multibeam_coverageAction> > action_client_;

  void actionDoneCallback(const actionlib::SimpleClientGoalState & state, const project11_nav_msgs::multibeam_coverageResult& result);
};

} // namespace project11_navigation


#endif
