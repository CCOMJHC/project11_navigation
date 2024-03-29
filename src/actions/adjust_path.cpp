#include <project11_navigation/actions/adjust_path.h>
#include <geometry_msgs/PoseStamped.h>
#include <project11_navigation/occupancy_grid.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>

namespace project11_navigation
{

AdjustPath::AdjustPath(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList AdjustPath::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("path"),
    BT::InputPort<int>("current_segment"),
    BT::InputPort<nav_msgs::Odometry>("odometry"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::InputPort<std::shared_ptr<OccupancyGrid> >("local_costmap"),
    BT::InputPort<double>("turn_radius"),

    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("adjusted_path"),
    BT::OutputPort<int>("adjusted_current_segment"),
    BT::OutputPort<double>("adjusted_segment_length"),
    BT::OutputPort<double>("adjusted_cross_track_error"),
    BT::OutputPort<double>("adjusted_along_track_progress"),
    BT::OutputPort<int>("adjusted_segment_count")
  };
}

BT::NodeStatus AdjustPath::tick()
{
  auto path_input = getInput<  std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("path");
  if(!path_input)
  {
    throw BT::RuntimeError(name(), " missing required input [path]: ", path_input.error() );
  }

  std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > adjusted_path;
  auto path = path_input.value();

  int total_segment_count = 0;

  if(path)
  {
    adjusted_path = std::make_shared<std::vector<geometry_msgs::PoseStamped> >();
    auto current_segment = getInput<int>("current_segment");
    if(current_segment)
    {
      if (current_segment.value() < path->size()-1)
      {
        auto p1 = path->at(current_segment.value());
        p1.pose.position.x +=5; // just testing!
        auto p2 = path->at(current_segment.value()+1);
        p2.pose.position.y += 5; // just testing!
        adjusted_path->push_back(p1);
        adjusted_path->push_back(p2);
      }
    }
    total_segment_count = adjusted_path->size()-1;
  }

  setOutput("adjusted_path", adjusted_path);
  setOutput("adjusted_current_segment", 0);
  setOutput("adjusted_cross_track_error", 0.0);
  setOutput("adjusted_along_track_progress", 0.0);
  setOutput("adjusted_segment_count", total_segment_count);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation 
