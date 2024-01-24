#include "project11_navigation/actions/update_current_task.h"
#include <project11_navigation/task_list.h>
#include <project11_navigation/task.h>

namespace project11_navigation
{

UpdateCurrentTask::UpdateCurrentTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList UpdateCurrentTask::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<TaskList> >("task_list"),
    BT::OutputPort<std::shared_ptr<Task> >("current_task"),
    BT::OutputPort<std::string>("current_task_type"),
    BT::OutputPort<std::string>("current_task_id"),
  };
}

BT::NodeStatus UpdateCurrentTask::tick()
{
  auto task_list_entry = getInput<std::shared_ptr<TaskList> >("task_list");
  if(!task_list_entry)
  {
    throw BT::RuntimeError("missing required input [task_list]: ", task_list_entry.error() );
  }

  auto task_list = task_list_entry.value();
  std::shared_ptr<Task> current_task;
  if(task_list)
  {
    auto todo_list = task_list->tasksByPriority(true);
    if(!todo_list.empty())
      current_task = todo_list.front();
  }
  setOutput("current_task", current_task);
  if(current_task)
  {
    setOutput("current_task_type", current_task->message().type);
    setOutput("current_task_id", current_task->message().id);
  }
  else
  {
    setOutput("current_task_type", "");
    setOutput("current_task_id", "");
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation
