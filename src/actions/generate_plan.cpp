#include <project11_navigation/actions/generate_plan.h>
#include <geometry_msgs/PoseStamped.h>
#include <project11_navigation/robot_capabilities.h>
#include <tf2/utils.h>

extern "C" {
#include "dubins_curves/dubins.h"
};


namespace project11_navigation
{

GeneratePlan::GeneratePlan(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList GeneratePlan::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::PoseStamped>("start_pose"),
    BT::InputPort<geometry_msgs::PoseStamped>("goal_pose"),
    BT::InputPort<std::string>("planner"),
    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("navigation_trajectory"),
    BT::InputPort<std::shared_ptr<RobotCapabilities> >("robot_capabilities"),
    BT::OutputPort<int>("current_navigation_segment")
  };
}

BT::NodeStatus GeneratePlan::tick()
{
  auto start_pose = getInput<geometry_msgs::PoseStamped>("start_pose");
  auto goal_pose = getInput<geometry_msgs::PoseStamped>("goal_pose");
  auto caps = getInput<std::shared_ptr<RobotCapabilities> >("robot_capabilities");
  if(start_pose && goal_pose && caps && caps.value())
  {
    double start[3];
    start[0] = start_pose.value().pose.position.x;
    start[1] = start_pose.value().pose.position.y;
    start[2] = tf2::getYaw(start_pose.value().pose.orientation);
    
    double target[3];
    target[0] = goal_pose.value().pose.position.x;
    target[1] = goal_pose.value().pose.position.y;
    target[2] = tf2::getYaw(goal_pose.value().pose.orientation);

    auto turn_radius = caps.value()->getTurnRadiusAtSpeed(caps.value()->default_velocity.linear.x);
    
    DubinsPath path;

    if(dubins_shortest_path(&path, start, target, turn_radius) == 0)
    {
      auto path_length = dubins_path_length(&path);
      auto step_size = turn_radius / 5.0;
      auto nav_trajectory = std::make_shared<std::vector<geometry_msgs::PoseStamped> >();
      nav_trajectory->push_back(start_pose.value());
      double current_length = 0.0;
      while(current_length < path_length)
      {
        double q[3];
        if(dubins_path_sample(&path, current_length, q) != 0)
          break;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = goal_pose.value().header.frame_id;
        pose.pose.position.x = q[0];
        pose.pose.position.y = q[1];
        tf2::Quaternion quat;
        quat.setRPY(0,0,q[2]);
        pose.pose.orientation = tf2::toMsg(quat);
        nav_trajectory->push_back(pose);
        current_length += step_size;
      }
      nav_trajectory->push_back(goal_pose.value());
      setOutput("navigation_trajectory", nav_trajectory);
      setOutput("current_navigation_segment", 0);
      return BT::NodeStatus::SUCCESS;
    }

  }
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation
