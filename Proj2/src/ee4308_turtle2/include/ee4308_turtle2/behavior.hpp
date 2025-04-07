#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ee4308_core/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::turtle2
{
    /**
     * The Behavior ROS Node that maintains subscribers and publishers for the Behavior class.
     */
    class Behavior : public rclcpp::Node
    {
    private:
        // parameters
        std::vector<double> waypoints_;
        double frequency_; // affects the plan rate.
        double reached_thres_;

        // topics, services, timers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_; // subscribe to ground truth in simulation.
        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr client_plan_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_stop_;
        std::shared_future<std::shared_ptr<nav_msgs::srv::GetPlan::Response>> request_plan_future_;
        rclcpp::TimerBase::SharedPtr timer_;

        // other states
        geometry_msgs::msg::PoseStamped rbt_pose_; // robot pose.
        size_t waypoint_idx_;
        bool plan_requested_;

    public:
        explicit Behavior(const std::string &name);

    private:
        void cbOdom(const nav_msgs::msg::Odometry msg);

        void cbTimer();

        void requestPlan(const geometry_msgs::msg::PoseStamped &rbt_pose, const geometry_msgs::msg::PoseStamped &waypoint_pose);

        // callback when a path is received. Not used.
        void cbReceivePlan(const rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future);
    };
}