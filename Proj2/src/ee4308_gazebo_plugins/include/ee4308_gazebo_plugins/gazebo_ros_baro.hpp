// Ported from ROS1 Noetic hector_quadrotor_gazebo_plugins, YK
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <ee4308_gazebo_plugins/sensor_model.h>
#include <ee4308_gazebo_plugins/update_timer.h>

// #include <geometry_msgs/PointStamped.h>
// #include <hector_uav_msgs/Altimeter.h>

#pragma once
namespace gazebo
{

class GazeboRosBaro : public ModelPlugin
{
public:
  GazeboRosBaro();
  virtual ~GazeboRosBaro();

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
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_altitude_;

  // ros::Publisher height_publisher_;
  // ros::Publisher altimeter_publisher_;

  // geometry_msgs::PointStamped height_;
  // hector_uav_msgs::Altimeter altimeter_;

  std::string namespace_;
  geometry_msgs::msg::PointStamped msg_altitude_;
  // std::string height_topic_;
  // std::string altimeter_topic_;
  std::string topic_altitude_;
  std::string link_name_;
  std::string frame_id_;

  double elevation_;
  double qnh_;

  SensorModel sensor_model_;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

// #ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
// #define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H

// // NOTE: Porting of CBQ functionality to ROS 2 is still pending.
// // #define USE_CBQ
// // #ifdef USE_CBQ
// // #include <ros/callback_queue.h>
// // #include <ros/advertise_options.h>
// // #endif

// #include <gazebo/common/Plugin.hh>
// #include <gazebo_ros/node.hpp>

// #include <rclcpp/rclcpp.hpp>
// #include <boost/thread/mutex.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <std_srvs/srv/empty.hpp>
// #include <hector_gazebo_plugins/srv/set_bias.hpp>
// #include <hector_gazebo_plugins/sensor_model.h>
// #include <hector_gazebo_plugins/update_timer.h>


// namespace gazebo
// {
//    class GazeboRosIMU : public ModelPlugin
//    {
//    public:
//       /// \brief Constructor
//       GazeboRosIMU();

//       /// \brief Destructor
//       virtual ~GazeboRosIMU();

//    protected:
//       virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
//       virtual void Reset();
//       virtual void Update();

//       rcl_interfaces::msg::SetParametersResult parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters);

//    private:
//       /// \brief The parent World
//       physics::WorldPtr world;

//       /// \brief The link referred to by this plugin
//       physics::LinkPtr link;

//       /// \brief pointer to ros node
//       gazebo_ros::Node::SharedPtr node_;
//       rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
//       rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr bias_pub_;

//       /// \brief ros message
//       sensor_msgs::msg::Imu imuMsg;
//       sensor_msgs::msg::Imu biasMsg;

//       /// \brief store link name
//       std::string link_name_;

//       /// \brief frame id
//       std::string frame_id_;

//       /// \brief topic name
//       std::string topic_;
//       std::string bias_topic_;

//       /// \brief allow specifying constant xyz and rpy offsets
// #if (GAZEBO_MAJOR_VERSION >= 8)
//       ignition::math::Pose3d offset_;
// #else
//       math::Pose offset_;
// #endif

//       /// \brief Sensor models
//       SensorModel3 accelModel;
//       SensorModel3 rateModel;
//       SensorModel yawModel;

//       /// \brief A mutex to lock access to fields that are used in message callbacks
//       boost::mutex lock;

//       /// \brief save current body/physics state
// #if (GAZEBO_MAJOR_VERSION >= 8)
//       ignition::math::Quaterniond orientation;
//       ignition::math::Vector3d velocity;
//       ignition::math::Vector3d accel;
//       ignition::math::Vector3d rate;
//       ignition::math::Vector3d gravity;
// #else
//       math::Quaternion orientation;
//       math::Vector3 velocity;
//       math::Vector3 accel;
//       math::Vector3 rate;
//       math::Vector3 gravity;
// #endif

//       /// \brief Gaussian noise generator
//       double GaussianKernel(double mu,double sigma);

//       /// \brief for setting ROS name space
//       std::string namespace_;

//       /// \brief call back when using service
//       bool ServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
//                            std::shared_ptr<std_srvs::srv::Empty::Response> res);

//       rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
//       std::string serviceName;

//       /// \brief Bias service callbacks
//       bool SetAccelBiasCallback(const std::shared_ptr<hector_gazebo_plugins::srv::SetBias::Request> req,
//                                 std::shared_ptr<hector_gazebo_plugins::srv::SetBias::Response> res);
//       bool SetRateBiasCallback(const std::shared_ptr<hector_gazebo_plugins::srv::SetBias::Request> req,
//                                std::shared_ptr<hector_gazebo_plugins::srv::SetBias::Response> res);
//       rclcpp::Service<hector_gazebo_plugins::srv::SetBias>::SharedPtr accelBiasService;
//       rclcpp::Service<hector_gazebo_plugins::srv::SetBias>::SharedPtr rateBiasService;

// /// \note Porting of CBQ functionality to ROS 2 is still pending.
// // #ifdef USE_CBQ
// //       ros::CallbackQueue callback_queue_;
// //       void CallbackQueueThread();
// //       boost::thread callback_queue_thread_;
// // #endif

//       UpdateTimer updateTimer;
//       event::ConnectionPtr updateConnection;

//       /// \brief Parameter changes callback
//       rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
//    };
// }

// #endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H