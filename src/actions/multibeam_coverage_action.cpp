#include <project11_navigation/actions/multibeam_coverage_action.h>
#include <project11_navigation/task.h>

namespace project11_navigation
{

MultibeamCoverageAction::MultibeamCoverageAction(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{
  
}

BT::PortsList MultibeamCoverageAction::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Task> >("task"),
    BT::InputPort<std::string>("ros_action")
  };
}

BT::NodeStatus MultibeamCoverageAction::onStart()
{
  auto task_bb = getInput<std::shared_ptr<Task> >("task");
  if(!task_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageAction node named ",name(), " missing required input [task]: ", task_bb.error() );
  }
  auto task = task_bb.value();
  if(task)
  {
    auto ros_action = getInput<std::string>("ros_action");
    if(!ros_action)
    {
      throw BT::RuntimeError("MultibeamCoverageAction node named ",name(), " missing required input [ros_action]: ", ros_action.error() );
    }
    action_client_ = std::make_shared<actionlib::SimpleActionClient<project11_nav_msgs::multibeam_coverageAction> >(ros_action.value());

    project11_nav_msgs::multibeam_coverageGoal goal;
    for(auto vertex: task->message().poses)
    {
      if(goal.survey_area.polygon.points.empty())
        goal.survey_area.header = vertex.header;
      geometry_msgs::Point32 p;
      p.x = vertex.pose.position.x;
      p.y = vertex.pose.position.y;
      p.z = vertex.pose.position.z;
      goal.survey_area.polygon.points.push_back(p);
    }
    //action_client_->sendGoal(goal, &MultibeamCoverageAction::actionDoneCallback);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MultibeamCoverageAction::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void MultibeamCoverageAction::onHalted()
{
  
}

}
