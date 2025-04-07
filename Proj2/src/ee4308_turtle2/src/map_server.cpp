#include "ee4308_turtle2/map_server.hpp"

namespace ee4308::turtle2
{
    MapServer::MapServer(const std::string &yaml_filepath, const std::string &name = "map_server") : rclcpp::Node(name), yaml_filepath_(yaml_filepath)
    {

        // get all parameters from the param server.
        initParam(this, "circumscribed_radius", circumscribed_radius_, 0.15);
        initParam(this, "inflation_radius", inflation_radius_, 0.5);
        initParam(this, "cost_exponent", cost_exponent_, 0.2);
        initParam(this, "min_cost", min_cost_, 0);
        initParam(this, "max_cost", max_cost_, 254);
        initParam(this, "obstacle_cost", obstacle_cost_, 255); // the cost to assign to the global costmap where an obstacle is found in "map" (obs layer)
        initParam(this, "obstacle_threshold", obstacle_threshold_, 50); // the cost in "map" (obs layer) when exceeded, is identified as an obstacle.

        // Initialize topics
        auto qos = rclcpp::ServicesQoS();
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // for latching the map message.
        pub_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
        pub_global_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap", qos);

        // Initialize Timer
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&MapServer::cbTimer, this));
    }

    void MapServer::cbTimer()
    {
        nav_msgs::msg::OccupancyGrid map = generateMap(yaml_filepath_);
        pub_map_->publish(map);

        nav_msgs::msg::OccupancyGrid global_costmap = generateGlobalCostmap(
            map, circumscribed_radius_, inflation_radius_, min_cost_, max_cost_, cost_exponent_);
        pub_global_costmap_->publish(global_costmap);

        timer_ = nullptr; // run once.
    }

    nav_msgs::msg::OccupancyGrid MapServer::generateMap(const std::string &yaml_filepath)
    {
        nav2_map_server::LoadParameters load_parameters = nav2_map_server::loadMapYaml(yaml_filepath); // map must exist.
        nav_msgs::msg::OccupancyGrid map;
        nav2_map_server::loadMapFromFile(load_parameters, map);
        return map; // refer to ee3305 package for old yaml and pgm to msg reader
    }

    nav_msgs::msg::OccupancyGrid MapServer::generateGlobalCostmap(const nav_msgs::msg::OccupancyGrid &map,
                                                                  double circumscribed_radius, double inflation_radius,
                                                                  int min_cost, int max_cost, double cost_exponent)
    {
        nav_msgs::msg::OccupancyGrid global_costmap = map; // copy
        global_costmap.data.clear();
        global_costmap.data.resize(map.data.size(), 0); // initialize all to zero ost

        // Fill costmap (exponential inflation costs)
        std::vector<MapServer::InflationMaskElement> mask = generateInflationMask(
            map.info.resolution,
            circumscribed_radius, inflation_radius,
            min_cost, max_cost, cost_exponent);
        const int width = global_costmap.info.width;
        const int height = global_costmap.info.height;

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                // from the closest neighbor cells, fill the cost if any obstacle cell lies within the mask.
                for (const MapServer::InflationMaskElement &rel : mask)
                {
                    const int my = y + rel.ry;
                    if (my < 0 || my > height)
                        continue; // skip if out of map
                    const int mx = x + rel.rx;
                    if (mx < 0 || mx > width)
                        continue; // skip if out of map

                    const int mkey = my * width + mx;
                    if (map.data[mkey] > obstacle_threshold_) // any value larger than 50 is treated as an obstacle.
                    {
                        const int key = y * width + x;
                        global_costmap.data[key] = rel.cost;
                        break; // closest obstacle found
                    }
                }
            }
        }
        return global_costmap;
    }

    std::vector<MapServer::InflationMaskElement> MapServer::generateInflationMask(
        double resolution, double circumscribed_radius, double inflation_radius,
        int min_cost, int max_cost, double cost_exponent)
    {
        if (inflation_radius < circumscribed_radius)
        {
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Circumscribed radius ("
                                   << circumscribed_radius
                                   << ") is larger than inflation radius ("
                                   << inflation_radius
                                   << "). Setting the value of the inflation radius to circumscribed radius.");
            inflation_radius = circumscribed_radius;
        }
        if (resolution <= 0)
            throw std::range_error("resolution must be a positive number");
        if (max_cost <= 0)
            throw std::range_error("max_cost must be a positive number");

        // generate mask
        std::vector<MapServer::InflationMaskElement> mask;
        double scale = (max_cost - min_cost) /
                       max_cost /
                       pow(inflation_radius - circumscribed_radius, cost_exponent);

        int c = ceil(inflation_radius / resolution);
        for (int my = -c; my <= c; ++my)
        {
            for (int mx = -c; mx <= c; ++mx)
            {
                const double dist = std::hypot((double)mx, (double)my) * resolution; // physical distance
                if (dist > inflation_radius)
                    continue;

                MapServer::InflationMaskElement inf;
                inf.dist = dist;
                inf.rx = mx;
                inf.ry = my;

                if (dist == 0)
                    inf.cost = obstacle_cost_;
                else if (dist < circumscribed_radius)
                    inf.cost = max_cost;
                else
                {
                    double pwr = pow(dist - circumscribed_radius, cost_exponent);
                    inf.cost = static_cast<char>(round(
                        max_cost * (1 - scale * pwr)));
                }
                mask.push_back(inf);
            }
        }
        // sort the mask. front is cheapest, back is furthest
        std::sort(mask.begin(), mask.end(),
                  [](const InflationMaskElement &a, const InflationMaskElement &b)
                  { return a.dist < b.dist; });

        return mask;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee4308::turtle2::MapServer>(std::string(argv[1])));
    rclcpp::shutdown();
    return 0;
}