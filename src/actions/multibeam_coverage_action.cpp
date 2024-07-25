#include <project11_navigation/actions/multibeam_coverage_action.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

MultibeamCoverageActionClient::MultibeamCoverageActionClient(std::shared_ptr<Task> task, const std::string& action_service)
{
  ROS_INFO_STREAM("Creating a client for " << action_service);
  action_client_ = std::make_shared<actionlib::SimpleActionClient<project11_nav_msgs::multibeam_coverageAction> >(action_service);

  if(!action_client_->waitForServer(ros::Duration(2.0)))
    ROS_INFO_STREAM("Timeout waiting for action server: " << action_service);

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
  action_client_->sendGoal(goal, boost::bind(&MultibeamCoverageActionClient::actionDoneCallback, this, _1, _2), boost::bind(&MultibeamCoverageActionClient::actionActiveCallback, this),
boost::bind(&MultibeamCoverageActionClient::actionFeedbackCallback, this, _1));

}

void MultibeamCoverageActionClient::actionDoneCallback(const actionlib::SimpleClientGoalState & state, const project11_nav_msgs::multibeam_coverageResultConstPtr& result)
{
  done_ = true;
}

void MultibeamCoverageActionClient::actionActiveCallback()
{
  ROS_INFO_STREAM("Multibeam Coverage Action active");
}

void MultibeamCoverageActionClient::actionFeedbackCallback(const project11_nav_msgs::multibeam_coverageFeedbackConstPtr& feedback)
{
  if(feedback->line_number != last_line_number_)
  {
    survey_lines_.push_back(feedback->current_line);
    last_line_number_ = feedback->line_number;
    adjustPathOrientations(survey_lines_.back().poses);
  }
}

int MultibeamCoverageActionClient::lineCount() const
{
  return survey_lines_.size();
}

int MultibeamCoverageActionClient::lastLineNumber() const
{
  return last_line_number_;
}

bool MultibeamCoverageActionClient::done() const
{
  return done_;
}

std::shared_ptr<actionlib::SimpleActionClient<project11_nav_msgs::multibeam_coverageAction> > MultibeamCoverageActionClient::actionClient() const
{
  return action_client_;
}



// *** SetGoal ***


MultibeamCoverageActionSetGoal::MultibeamCoverageActionSetGoal(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  
}

BT::PortsList MultibeamCoverageActionSetGoal::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Task> >("task"),
    BT::InputPort<std::string>("ros_action"),
    BT::OutputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client")
  };
}

BT::NodeStatus MultibeamCoverageActionSetGoal::tick()
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

    setOutput("action_client", std::make_shared<MultibeamCoverageActionClient>(task, ros_action.value()));
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// *** UpdateTask ***

MultibeamCoverageActionUpdateTask::MultibeamCoverageActionUpdateTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  
}

BT::PortsList MultibeamCoverageActionUpdateTask::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client"),
    BT::InputPort<std::shared_ptr<Task> >("task"),
  };
}

BT::NodeStatus MultibeamCoverageActionUpdateTask::tick()
{
  auto action_client_bb = getInput<std::shared_ptr<MultibeamCoverageActionClient> >("action_client");
  if(!action_client_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionUpdateTask node named ",name(), " missing required input [action_client]: ", action_client_bb.error() );
  }

  auto task_bb = getInput<std::shared_ptr<Task> >("task");
  if(!task_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionUpdateTask node named ",name(), " missing required input [task]: ", task_bb.error() );
  }
  auto action_client = action_client_bb.value();
  auto task = task_bb.value();
  if(action_client && task)
  {
    while(action_client->survey_lines_.size() > task->children().tasks().size())
    {
      auto line_index = task->children().tasks().size();
      auto new_child_task = task->createChildTaskBefore();
      std::stringstream task_name;
      task_name << "line_" << line_index;
      task->setChildID(new_child_task, task_name.str());
      auto info = new_child_task->message();
      info.type = "survey_line";
      info.poses = action_client->survey_lines_[line_index].poses;
      new_child_task->update(info);
      ROS_INFO_STREAM("New task: " << new_child_task->message());
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// *** Cancel ***

MultibeamCoverageActionCancel::MultibeamCoverageActionCancel(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  
}

BT::PortsList MultibeamCoverageActionCancel::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client")
  };
}


BT::NodeStatus MultibeamCoverageActionCancel::tick()
{
  auto action_client_bb = getInput<std::shared_ptr<MultibeamCoverageActionClient> >("action_client");
  if(!action_client_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionCancel node named ",name(), " missing required input [action_client]: ", action_client_bb.error() );
  }
  action_client_bb.value()->action_client_->cancelGoal();
  return BT::NodeStatus::SUCCESS;
}

// *** DoneCondition ***

MultibeamCoverageActionDoneCondition::MultibeamCoverageActionDoneCondition(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{
  
}

BT::PortsList MultibeamCoverageActionDoneCondition::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client")
  };
}

BT::NodeStatus MultibeamCoverageActionDoneCondition::tick()
{
  auto action_client_bb = getInput<std::shared_ptr<MultibeamCoverageActionClient> >("action_client");
  if(!action_client_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionDoneCondition node named ",name(), " missing required input [action_client]: ", action_client_bb.error() );
  }
  if (action_client_bb.value()->done_)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}


}
