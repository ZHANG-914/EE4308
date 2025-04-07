// Ported from ROS2 Humble hector_gazebo_plugins, YK
#pragma once
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <ee4308_gazebo_plugins/sensor_model.h>
#include <ee4308_gazebo_plugins/update_timer.h>

namespace gazebo
{

class GazeboRosMagnetic : public ModelPlugin
{
public:
  GazeboRosMagnetic();
  virtual ~GazeboRosMagnetic();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_magnetic_field_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vector3_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  bool use_magnetic_field_msgs_;

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d magnetic_field_world_;
#else
  gazebo::math::Vector3 magnetic_field_world_;
#endif

  std::string namespace_;
  std::string topic_;
  std::string link_name_;
  std::string frame_id_;

  double magnitude_;
  double reference_heading_;
  double declination_;
  double inclination_;

  SensorModel3 sensor_model_;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo
