#include "project11_navigation/actions/task_list_updater.h"
#include <project11_navigation/task_list.h>

namespace project11_navigation
{

TaskListUpdater::TaskListUpdater(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList TaskListUpdater::providedPorts()
{
  return {
    BT::BidirectionalPort<std::shared_ptr<std::vector<project11_nav_msgs::TaskInformation> > >("task_messages"),
    BT::BidirectionalPort<std::shared_ptr<TaskList> >("task_list")
  };
}

BT::NodeStatus TaskListUpdater::tick()
{
  auto task_messages = getInput<std::shared_ptr<std::vector<project11_nav_msgs::TaskInformation> > >("task_messages");
  if(task_messages)
  {
    if(task_messages.value())
    {
      std::shared_ptr<TaskList> task_list;
      auto task_list_entry = getInput<std::shared_ptr<TaskList> >("task_list");
      if(task_list_entry)
      {
        task_list = task_list_entry.value();

      }
      if(!task_list)
      {
        task_list = std::make_shared<TaskList>();
        setOutput("task_list", task_list);
      }
      task_list->update(*task_messages.value());
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation
