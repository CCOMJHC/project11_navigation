#ifndef PROJECT11_NAVIGATION_ACTIONS_MULTIBEAM_COVERAGE_ACTION_H
#define PROJECT11_NAVIGATION_ACTIONS_MULTIBEAM_COVERAGE_ACTION_H

#include <behaviortree_cpp/bt_factory.h>
#include <actionlib/client/simple_action_client.h>
#include <project11_nav_msgs/multibeam_coverageAction.h>
#include <project11_navigation/task.h>

namespace project11_navigation
{


class MultibeamCoverageActionSetGoal: public BT::SyncActionNode
{
public:
  MultibeamCoverageActionSetGoal(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class MultibeamCoverageActionUpdateTask: public BT::SyncActionNode
{
public:
  MultibeamCoverageActionUpdateTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class MultibeamCoverageActionCancel: public BT::SyncActionNode
{
public:
  MultibeamCoverageActionCancel(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class MultibeamCoverageActionDoneCondition: public BT::ConditionNode
{
public:
  MultibeamCoverageActionDoneCondition(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class MultibeamCoverageActionClient
{
public:
  MultibeamCoverageActionClient(std::shared_ptr<Task> task, const std::string& action_service);

  int lineCount() const;
  int lastLineNumber() const;
  bool done() const;

  std::shared_ptr<actionlib::SimpleActionClient<project11_nav_msgs::multibeam_coverageAction> > actionClient() const;

private:
  friend class MultibeamCoverageActionUpdateTask;
  friend class MultibeamCoverageActionCancel;
  friend class MultibeamCoverageActionDoneCondition;

  std::shared_ptr<actionlib::SimpleActionClient<project11_nav_msgs::multibeam_coverageAction> > action_client_;

  void actionActiveCallback();
  void actionDoneCallback(const actionlib::SimpleClientGoalState & state, const project11_nav_msgs::multibeam_coverageResultConstPtr& result);
  void actionFeedbackCallback(const project11_nav_msgs::multibeam_coverageFeedbackConstPtr& feedback);

  std::vector<nav_msgs::Path> survey_lines_;
  int last_line_number_ = -1;
  bool done_ = false;

};


} // namespace project11_navigation


#endif
