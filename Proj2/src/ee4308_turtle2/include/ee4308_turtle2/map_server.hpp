#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav2_map_server/map_io.hpp"
#include "ee4308_core/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::turtle2
{
    /**
     * Serves the map by publishing into the `map` and the inflated layer into the `global_costmap` topics.
     */
    class MapServer : public rclcpp::Node
    {
    private:
        struct InflationMaskElement
        {
            char cost;   // relative cost given the distance.
            int ry;      // relative y coordinate (in terms of grid cell)
            int rx;      // relative x coordinate  (in terms of grid cell)
            double dist; // physical distance
        };

        std::string yaml_filepath_;
        double circumscribed_radius_;
        double inflation_radius_;
        int min_cost_;
        int max_cost_;
        int obstacle_cost_;
        int obstacle_threshold_;
        double cost_exponent_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;            
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_global_costmap_; 
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        explicit MapServer(const std::string &yaml_filepath, const std::string &name);

    private:
        /** Publishes the obstacle layer into the `map` topic and inflation layer into the `global_costmap` topic. Run only once. */
        void cbTimer();

        /** Generates the obstacle layer. */
        nav_msgs::msg::OccupancyGrid generateMap(const std::string &yaml_filepath);

        /** Generates the inflation layer. */
        nav_msgs::msg::OccupancyGrid generateGlobalCostmap(const nav_msgs::msg::OccupancyGrid &map,
                                                           double circumscribed_radius, double inflation_radius,
                                                           int min_cost, int max_cost, double cost_exponent);

        /** Generates an inflation mask which is used to generate the inflation layer from the obstacle layer. */
        std::vector<InflationMaskElement> generateInflationMask(
            double resolution, double circumscribed_radius, double inflation_radius,
            int min_cost, int max_cost, double cost_exponent);
    };
}