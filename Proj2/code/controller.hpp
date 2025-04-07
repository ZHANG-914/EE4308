#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "ee4308_core/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::drone
{
    class Controller : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Odometry odom_;
        nav_msgs::msg::Path plan_;

        bool use_ground_truth_;
        bool enable_;
        double frequency_;
        double lookahead_distance_;
        double max_xy_vel_;
        double max_z_vel_;
        double yaw_vel_;
        double kp_xy_;
        double kp_z_;

    public:
        explicit Controller(const std::string &name);

    private:
        void cbOdom(const nav_msgs::msg::Odometry msg);

        void cbPlan(const nav_msgs::msg::Path msg);

        void cbTimer();

        void publishCmdVel(double x_vel, double y_vel, double z_vel, double yaw_vel);
    };
}