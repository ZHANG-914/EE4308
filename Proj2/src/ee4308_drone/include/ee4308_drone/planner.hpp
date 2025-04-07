#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "ee4308_core/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::drone
{

    class Planner : public rclcpp::Node
    {
    private:
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr service_plan_;

        double interpolation_distance_;

    public:
        explicit Planner(const std::string &name);

        void cbServicePlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                           std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);
    };
}