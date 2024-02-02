#include <project11_navigation/actions/baxevani_controller.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "project11/utils.h"
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>

namespace project11_navigation
{

BaxevaniController::BaxevaniController(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{
  ros::NodeHandle nh("~");


  acceleration_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("baxevani/debug/acceleration", 1);
  angular_acceleration_pub_ = nh.advertise<std_msgs::Float32>("baxevani/debug/angular_acceleration", 1);
  u_pub_ = nh.advertise<std_msgs::Float32>("baxevani/debug/u", 1);
  a_pub_= nh.advertise<std_msgs::Float32>("baxevani/debug/a", 1);
  alpha_pub_ = nh.advertise<std_msgs::Float32>("baxevani/debug/alpha", 1);

}

BT::PortsList BaxevaniController::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("navigation_path"),
    BT::InputPort<int>("current_navigation_segment"),
    BT::InputPort<nav_msgs::Odometry>("odometry"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::InputPort<double>("target_speed"),
    BT::InputPort<double>("ricatti_parameter"),
    BT::InputPort<double>("px_gain"),
    BT::InputPort<double>("pw_gain"),
    BT::InputPort<double>("delta"),
    BT::InputPort<double>("maximum_dt"),
    BT::OutputPort<geometry_msgs::TwistStamped>("command_velocity"),
  };
}

BT::NodeStatus BaxevaniController::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BaxevaniController::onRunning()
{
  auto navigation_path_bb = getInput<std::shared_ptr<std::vector< geometry_msgs::PoseStamped> > >("navigation_path");
  if(!navigation_path_bb)
  {
    throw BT::RuntimeError("missing required input [navigation_path]: ", navigation_path_bb.error() );
  }
  auto navigation_path = navigation_path_bb.value();

  int segment_count = std::max<int>(0,navigation_path->size()-1);

  auto current_segment = getInput<int>("current_navigation_segment");
  if(!current_segment)
  {
    throw BT::RuntimeError("missing required input [current_navigation_segment]: ", current_segment.error() );
  }

  if(current_segment.value() < 0 || current_segment.value() > segment_count)
    return BT::NodeStatus::FAILURE;

  // We're done if we are at the segment past the last one
  if(current_segment.value() == segment_count)
    return BT::NodeStatus::SUCCESS;

  auto odom = getInput<nav_msgs::Odometry>("odometry");
  if(!odom)
  {
    throw BT::RuntimeError("missing required input [odometry]: ", odom.error() );
  }

  auto tf_buffer = getInput<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer");
  if(!tf_buffer && tf_buffer.value())
  {
    throw BT::RuntimeError("missing required input [tf_buffer]: ", tf_buffer.error() );
  }

  double target_speed = 0.0;
  auto target_speed_bb = getInput<double>("target_speed");
  if(!target_speed_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [target_speed]: ", target_speed_bb.error() );
  }
  target_speed = target_speed_bb.value();

  auto ricatti_parameter = getInput<double>("ricatti_parameter");
  if(!ricatti_parameter)
  {
    throw BT::RuntimeError(name(), " missing required input [ricatti_parameter]: ", ricatti_parameter.error() );
  }

  auto px_gain_bb = getInput<double>("px_gain");
  if(!px_gain_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [px_gain]: ", px_gain_bb.error() );
  }

  auto pw_gain_bb = getInput<double>("pw_gain");
  if(!pw_gain_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [pw_gain]: ", pw_gain_bb.error() );
  }

  auto delta_bb = getInput<double>("delta");
  if(!delta_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [delta]: ", delta_bb.error() );
  }

  auto maximum_dt = getInput<double>("maximum_dt");
  if(!maximum_dt)
  {
    throw BT::RuntimeError(name(), " missing required input [maximum_dt]: ", maximum_dt.error() );
  }

  geometry_msgs::TransformStamped base_to_map;
  try
  {
    base_to_map = tf_buffer.value()->lookupTransform(navigation_path->front().header.frame_id , odom.value().child_frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("FollowPathCommand node named " << name() << " Error getting path to base_frame transform: " << ex.what());
    return BT::NodeStatus::FAILURE;
  }

  auto p1 = (*navigation_path)[current_segment.value()];
  auto p2 = (*navigation_path)[current_segment.value()+1];

  auto segment_dx = p2.pose.position.x - p1.pose.position.x;
  auto segment_dy = p2.pose.position.y - p1.pose.position.y;

  project11::AngleRadians segment_azimuth = atan2(segment_dy, segment_dx);
  auto segment_distance = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

  // vehicle distance and azimuth relative to the segment's start point
  double dx = p1.pose.position.x - base_to_map.transform.translation.x;
  double dy = p1.pose.position.y - base_to_map.transform.translation.y;
  auto vehicle_distance = sqrt(dx*dx+dy*dy);

  project11::AngleRadians vehicle_azimuth = atan2(-dy, -dx);

  auto error_azimuth = vehicle_azimuth - segment_azimuth;
     
  auto sin_error_azimuth = sin(error_azimuth);
  auto cos_error_azimuth = cos(error_azimuth);

  // Distance traveled along the line.
  auto progress = vehicle_distance*cos_error_azimuth;

  auto cross_track_error = vehicle_distance*sin_error_azimuth;

  
  project11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);

  project11::AngleRadians target_heading = segment_azimuth;

  project11::AngleRadians theta = project11::AngleRadiansZeroCentered(target_heading - heading).value();

  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.stamp = odom.value().header.stamp;
  cmd_vel.header.frame_id = odom.value().child_frame_id;

  auto dt = (odom.value().header.stamp - last_odom_.header.stamp).toSec();
  if(dt <= maximum_dt.value() && dt > 0.0)
  {
    auto twist = odom.value().twist.twist;

    double sensor_velx = twist.linear.x;
    double sensor_vely = twist.linear.y;
    double sensor_velw = twist.angular.z;

    auto last_twist = last_odom_.twist.twist;

    double prev_sensor_velx = last_twist.linear.x;
    double prev_sensor_vely = last_twist.linear.y;
    double prev_sensor_velw = last_twist.angular.z;

    double current_y = cross_track_error;

    double r = ricatti_parameter.value();
    double px_gain = px_gain_bb.value();
    double pw_gain = pw_gain_bb.value();
    
    double v_d = target_speed;
    double delta = delta_bb.value();

    double acc_x = (sensor_velx - prev_sensor_velx) / dt; //Acceleration on the surge axis
    double acc_y = (sensor_vely - prev_sensor_vely) / dt; //Acceleration on the sway axis
    double acc_w = (sensor_velw - prev_sensor_velw) / dt; //Acceleration on the sway axis

    geometry_msgs::Vector3Stamped acceleration_msg;
    acceleration_msg.header = odom.value().header;
    acceleration_msg.header.frame_id = odom.value().child_frame_id;
    acceleration_msg.vector.x = acc_x;
    acceleration_msg.vector.y = acc_y;

    acceleration_pub_.publish(acceleration_msg);

    std_msgs::Float32 angular_acceleration_msg;
    angular_acceleration_msg.data = acc_w;

    angular_acceleration_pub_.publish(angular_acceleration_msg);
    

    double u = -(current_y + delta * sin(theta)) / sqrt(r) -
        (sensor_velx * sin(-theta) + (sensor_vely + delta * sensor_velw)
         * cos(-theta)) * sqrt(2.0) / pow(r, (1.0 / 4.0)); // Optimal feedback law for the system

    double a = 1 / r * (v_d - sensor_velx) * cos(theta) - u * sin(theta) +
        delta * pow(sensor_velw, 2.0) + (pow(sin(heading), 2.0) -
        pow(cos(heading), 2.0)) * sensor_vely * sensor_velw; // Linear acceleration on the x axis (axis of motion)

    double alpha = 1 / (r * delta) * (v_d - sensor_velx) * sin(theta) + u * 
        cos(theta) / delta - sensor_velx * sensor_velw / delta - 1 / delta *
        acc_y + 2 * cos(heading) * sin(heading) * sensor_vely * sensor_velw / delta; // Angular acceleration around z axis

    std_msgs::Float32 u_msg;
    u_msg.data = u;
    u_pub_.publish(u_msg);

    std_msgs::Float32 a_msg;
    a_msg.data = a;
    a_pub_.publish(a_msg);

    std_msgs::Float32 alpha_msg;
    alpha_msg.data = alpha;
    alpha_pub_.publish(alpha_msg);

    cmd_vel.twist.linear.x = 1.0*(prev_sensor_velx + acc_x*dt + px_gain*(a-acc_x)*pow(dt,2.0));
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 1.0*(prev_sensor_velw + acc_w*dt + pw_gain*(alpha-acc_w)*pow(dt,2.0));

  }

  last_odom_ = odom.value();

  setOutput("command_velocity", cmd_vel);
  return BT::NodeStatus::RUNNING;
}


void BaxevaniController::onHalted()
{
  
}


} // namespace project11_navigation
