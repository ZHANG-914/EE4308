#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/controller.hpp"
#include "tf2/utils.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ee4308_core/core.hpp"

#pragma once

namespace ee4308::turtle
{
    class Controller : public nav2_core::Controller
    {
    public:
        Controller() = default;
        ~Controller() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void cleanup() override;
        void activate() override;
        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity,
            nav2_core::GoalChecker *goal_checker) override;

        void setPlan(const nav_msgs::msg::Path &path) override;
        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

    protected:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;

        // parameters
        double desired_linear_vel_;
        double desired_lookahead_dist_;
        double max_angular_vel_;
        double max_linear_vel_;
        double xy_goal_thres_;
        double yaw_goal_thres_;

        // topics 
        nav_msgs::msg::Path global_plan_;
        std::vector<float> scan_ranges_;
        // rclcpp::Subscription<some msg type>::SharedPtr sub_scan_;
        // void some callback(some msg type::SharedPtr msg);

        // other "private" functions
        geometry_msgs::msg::TwistStamped writeCmdVel(double linear_vel, double angular_vel);
    };

}