#include <iostream>
#include <algorithm>
#include <queue>
#include <cmath>
#include <chrono>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "ee4308_core/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::turtle2
{
    class Planner : public rclcpp::Node
    {
    private:
        // Using more complicated implementations to avoid copying by future students.
        using OpenListNode = std::pair<double, unsigned int>;
        struct OpenListComparator
        {
            bool operator()(const OpenListNode &l, const OpenListNode &r) const { return l.first > r.first; };
        };
        using OpenList = std::priority_queue<OpenListNode, std::deque<OpenListNode>, OpenListComparator>;
        using PlannerNode = std::tuple<double, unsigned int, bool>;
        static constexpr char COST_ = 0;
        static constexpr char PARENT_ = 1;
        static constexpr char EXPANDED_ = 2;

        // params
        unsigned char max_access_cost_;

        // publishers and subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap_; // publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;                       // publisher
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr service_plan_;                  // service

        std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

    public:
        /**
         * Constructor for the Planner ROS Node.
         * @param name name of node.
         */
        explicit Planner(const std::string &name) ;

    private:
        void cbMap(nav_msgs::msg::OccupancyGrid msg);

        void cbServicePlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                         std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);

        nav_msgs::msg::Path plan(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal);
    };
}