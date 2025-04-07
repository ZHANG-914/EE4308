#include "ee4308_turtle2/planner.hpp"

namespace ee4308::turtle2
{
    Planner::Planner(const std::string &name = "planner"): rclcpp::Node(name)
    {
        // states
        costmap_ = nullptr;

        // topics
        pub_path_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::ServicesQoS());

        auto qos = rclcpp::ServicesQoS();
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // for getting the latched map message.
        sub_global_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap", qos,
                                                                                std::bind(&Planner::cbMap, this, std::placeholders::_1));

        // params
        int t;
        initParam(this, "max_access_cost", t, 254);
        max_access_cost_ = t;

        // services
        service_plan_ = create_service<nav_msgs::srv::GetPlan>(
            "get_plan",
            std::bind(&Planner::cbServicePlan, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);
    }

    void Planner::cbMap(nav_msgs::msg::OccupancyGrid msg)
    {
        costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(msg);
    }

    /**
     * The service callback to respond with the path
     */
    void Planner::cbServicePlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                              std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        if (costmap_ == nullptr)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Empty path returned because global_costmap has not been published.");
            response->plan.header.frame_id = "map";
            response->plan.header.stamp = this->now();
            response->plan.poses.clear();
            return;
        }

        // RCLCPP_INFO_STREAM(this->get_logger(), "Plan request");
        response->plan.header.frame_id = "map";
        response->plan.header.stamp = this->now();

        // find the shortest path and store the path within the class.
        // std::cout << "Running Planner..." << std::endl;
        nav_msgs::msg::Path path = plan(request->start, request->goal);
        // std::cout << "Run complete." << std::endl;

        // publish the path
        pub_path_->publish(path);

        // respond with the path
        response->plan = path;
    }

    nav_msgs::msg::Path Planner::plan(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal)
    {
        // prepare plan
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = "map";

        // get the grid cell coordinates
        int t1, t2;
        costmap_->worldToMapEnforceBounds(
            start.pose.position.x, start.pose.position.y,
            t1, t2);
        unsigned int start_mx = t1, start_my = t2;
        costmap_->worldToMapEnforceBounds(
            goal.pose.position.x, goal.pose.position.y,
            t1, t2);
        unsigned int goal_mx = t1, goal_my = t2;

        // inits
        OpenList pq_;
        std::vector<PlannerNode> nodes_(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY(), {INFINITY, 0, false});

        // lambdas
        auto inMap = [&](const unsigned int &mx, const unsigned int &my)
        { return mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY(); };
        auto getCost2 = [&](const unsigned int &mx, const unsigned int &my)
        { return std::hypot((double)mx - goal_mx, (double)my - goal_my); };
        auto queue = [&](const unsigned int &idx, const double &queued_cost)
        { pq_.push({queued_cost, idx}); };

        // start node
        unsigned int idx = costmap_->getIndex(start_mx, start_my);
        PlannerNode *node = &nodes_[idx];
        std::get<COST_>(*node) = 0;
        queue(idx, getCost2(start_mx, start_my));

        // main loop
        while (rclcpp::ok() && !pq_.empty())
        {
            // poll
            idx = pq_.top().second;
            unsigned int mx, my;
            costmap_->indexToCells(idx, mx, my);
            pq_.pop();
            // std::cout << "========poll (" << idx << "," << mx << "," << my <<  ")" << std::endl;

            // path found
            if (mx == goal_mx && my == goal_my)
            {
                std::vector<std::pair<unsigned int, unsigned int>> coords;
                unsigned int mx, my;
                do
                {
                    idx = std::get<1>(nodes_[idx]);
                    costmap_->indexToCells(idx, mx, my);
                    coords.push_back({mx, my});
                } while (mx != start_mx || my != start_my);
                std::reverse(coords.begin(), coords.end());
                // std::cout << "coords suze" << coords.size() << std::endl;

                // std::cout << "path found (world)[" << (coords.size() / 2) << "]: ";
                for (const auto &[mx, my] : coords)
                { // goal pose is skipped.
                    geometry_msgs::msg::PoseStamped pose;
                    costmap_->mapToWorld(mx, my, pose.pose.position.x, pose.pose.position.y);
                    pose.pose.orientation.w = 1.0;
                    path.poses.push_back(pose);
                    // std::cout << "(" << pose.pose.position.x << "," << pose.pose.position.y << ") ";
                }
                // std::cout << std::endl;

                goal.header.frame_id = "";
                goal.header.stamp = rclcpp::Time();
                path.poses.push_back(goal); // add goal to preserve requested orientation and position.

                path.header.stamp = this->now();

                return path;
            }

            // skip if visited
            node = &nodes_[idx];
            if (std::get<EXPANDED_>(*node))
                continue;
            std::get<EXPANDED_>(*node) = true;

            // check neighbors
            constexpr std::array<int, 16> coords =
                {1, 0, 1, 1, 0, 1, -1, 1, -1, 0, -1, -1, 0, -1, 1, -1};
            for (unsigned int d = 0; d < 16; d += 2)
            {
                unsigned int nb_mx = mx + coords[d];
                unsigned int nb_my = my + coords[d + 1];

                if (!inMap(nb_mx, nb_my))
                {
                    // std::cout << "nb oom(" << nb_mx << "," << nb_my << ")" << std::endl;
                    continue; // oom
                }

                unsigned int nb_idx = costmap_->getIndex(nb_mx, nb_my);
                PlannerNode *nb_node = &nodes_[nb_idx];
                if (std::get<EXPANDED_>(*nb_node))
                {
                    // std::cout << "nb exp" << std::endl;
                    continue; // exp
                }

                unsigned char nb_cell_cost = costmap_->getCost(nb_mx, nb_my);
                if (nb_cell_cost > max_access_cost_)
                {
                    // std::cout << "costly" << nb_cell_cost << ">" << max_access_cost_ << std::endl;
                    continue; // costly
                }

                double new_cost = std::get<COST_>(*node) + std::hypot(coords[d], coords[d + 1]) * (nb_cell_cost + 1);
                if (new_cost < std::get<COST_>(*nb_node))
                {
                    std::get<PARENT_>(*nb_node) = idx;
                    std::get<COST_>(*nb_node) = new_cost;
                    queue(nb_idx, getCost2(nb_mx, nb_my) + new_cost);
                }
                else
                {
                    // std::cout << "nb more exp" << std::endl;
                }
            }
        }
        RCLCPP_WARN_STREAM(this->get_logger(), "No path found!");
        path.header.stamp = this->now();
        return path;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee4308::turtle2::Planner>());
    rclcpp::shutdown();
    return 0;
}