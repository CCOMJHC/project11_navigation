#include "project11_navigation/actions/update_state.h"
#include <project11_navigation/context.h>
#include <project11_navigation/robot.h>
#include <project11_navigation/robot_capabilities.h>
#include <nav_msgs/Odometry.h>

namespace project11_navigation
{

UpdateState::UpdateState(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList UpdateState::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Context> >("context"),
    BT::OutputPort<std::string>("piloting_mode"),
    BT::OutputPort<std::string>("base_frame"),
    BT::OutputPort<nav_msgs::Odometry>("odometry"),
    BT::OutputPort<geometry_msgs::PoseStamped>("current_pose"),
    BT::OutputPort<geometry_msgs::TwistStamped>("command_velocity"),
    BT::OutputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::OutputPort<std::shared_ptr<visualization_msgs::MarkerArray> >("marker_array"),
    BT::InputPort<geometry_msgs::Polygon>("robot_footprint"),
    BT::OutputPort<std::shared_ptr<OccupancyGrid> >("local_costmap")
  };
}

BT::NodeStatus UpdateState::tick()
{
  auto context = getInput<std::shared_ptr<Context> >("context");

  if(context.value()->robot().enabled())
    setOutput("piloting_mode", "autonomous");
  else
    setOutput("piloting_mode", "not_autonomous");

  setOutput("base_frame", context.value()->robot().baseFrame());
  setOutput("odometry", context.value()->robot().odometry());
  geometry_msgs::PoseStamped current_pose;
  current_pose.header = context.value()->robot().odometry().header;
  current_pose.pose = context.value()->robot().odometry().pose.pose;
  setOutput("current_pose", current_pose);
  geometry_msgs::TwistStamped ts;
  ts.header.frame_id = context.value()->robot().baseFrame();
  setOutput("command_velocity", ts);
  setOutput("tf_buffer", context.value()->tfBuffer());

  auto marker_array = std::make_shared<visualization_msgs::MarkerArray>();
  auto footprint = getInput<geometry_msgs::Polygon>("robot_footprint");
  if(footprint)
  {
    context.value()->robot().updateMarkers(*marker_array, footprint.value());
  }
  setOutput("marker_array", marker_array);

  setOutput("local_costmap", context.value()->environment().localCostmap());

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation
