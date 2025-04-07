#include "ee4308_drone/planner.hpp"

namespace ee4308::drone
{
    Planner::Planner(const std::string &name = "planner")
        : Node(name)
    {

        // parameters
        initParam(this, "interpolation_distance", interpolation_distance_, 0.1);

        // topics
        pub_path_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::ServicesQoS());

        // services
        service_plan_ = this->create_service<nav_msgs::srv::GetPlan>(
            "get_plan", std::bind(&Planner::cbServicePlan, this, std::placeholders::_1, std::placeholders::_2));
    }

    void Planner::cbServicePlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                           std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        response->plan.header.frame_id = "map";
        response->plan.header.stamp = this->now();

        double dx = request->goal.pose.position.x - request->start.pose.position.x;
        double dy = request->goal.pose.position.y - request->start.pose.position.y;
        double dz = request->goal.pose.position.z - request->start.pose.position.z;

        double distance = std::hypot(dx, dy, dz);
        int steps = std::floor(distance / interpolation_distance_); // floored.
        for (int i = 0; i < steps; ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            double fraction = i * interpolation_distance_ / distance;
            pose.pose.position.x = request->start.pose.position.x + fraction * dx;
            pose.pose.position.y = request->start.pose.position.y + fraction * dy;
            pose.pose.position.z = request->start.pose.position.z + fraction * dz;
            response->plan.poses.push_back(pose);
        }
        response->plan.poses.push_back(request->goal);

        // publish the path
        pub_path_->publish(response->plan);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ee4308::drone::Planner>());
    rclcpp::shutdown();
    return 0;
}