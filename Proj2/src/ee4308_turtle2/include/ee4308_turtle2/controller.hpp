#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "ee4308_core/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::turtle2
{
    /**
     * The Controller ROS Node that maintains subscribers and publishers for the  class.
     */
    class Controller : public rclcpp::Node
    {
    private:
        // parameters
        double frequency_;
        double kp_lin_;
        double kp_ang_;
        double max_lin_vel_;
        double max_ang_vel_;
        double xy_tolerance_;
        double yaw_tolerance_;
        double lookahead_distance_;
        bool enable_;

        // states
        geometry_msgs::msg::PoseStamped rbt_pose_;
        nav_msgs::msg::Path plan_;

        // topics, services, timers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;   // subscribe to ground truth in simulation.
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;       // subscribe to ground truth in simulation.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_; // subscribe to ground truth in simulation.
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        explicit Controller(const std::string &name);

    private:
        void cbOdom(const nav_msgs::msg::Odometry msg);

        void cbPlan(const nav_msgs::msg::Path msg);

        void cbTimer();

        void publishCmdVel(const double &lin_vel, const double &ang_vel);
    };
}