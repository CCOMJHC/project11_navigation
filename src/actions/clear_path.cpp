#include <project11_navigation/actions/clear_path.h>
#include <geometry_msgs/PoseStamped.h>

namespace project11_navigation
{

ClearPath::ClearPath(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList ClearPath::providedPorts()
{
  return {
    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("navigation_path")
  };
}

BT::NodeStatus ClearPath::tick()
{

  setOutput("navigation_path", std::shared_ptr<std::vector<geometry_msgs::PoseStamped> >());
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation
