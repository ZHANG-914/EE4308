#include "ee4308_turtle2/behavior.hpp"

namespace ee4308::turtle2
{
    Behavior::Behavior(const std::string &name = "behavior") : Node(name)
    {
        // states
        waypoint_idx_ = 0;
        plan_requested_ = false;

        // parameters
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "reached_thres", reached_thres_, 0.1);
        initParamDoubleArray(this, "waypoints", waypoints_, {});

        // topics
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&Behavior::cbOdom, this, std::placeholders::_1));
        pub_stop_ = this->create_publisher<std_msgs::msg::Empty>(
            "stop", rclcpp::SensorDataQoS());

        // services
        client_plan_ = this->create_client<nav_msgs::srv::GetPlan>("get_plan");

        // wait for topics to publish
        rclcpp::Rate rate(2.0);
        while (rclcpp::ok() && (rbt_pose_.header.stamp.sec == 0))
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for subscribed topics");
            rate.sleep();
            rclcpp::spin_some(this->get_node_base_interface());
        }

        // Initialize main loop
        timer_ = this->create_wall_timer(1s / frequency_,
                                         std::bind(&Behavior::cbTimer, this));
    }

    void Behavior::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        rbt_pose_.pose = msg.pose.pose;
        rbt_pose_.header = msg.header;
    }

    void Behavior::cbTimer()
    {
        // all waypoints processed or no waypoints at all.
        if (waypoint_idx_ >= waypoints_.size())
        {
            pub_stop_->publish(std_msgs::msg::Empty());
            timer_ = nullptr; // remove itself.
            return; // reached end of path.
        }

        // get the current waypoint.
        const double &waypoint_x = waypoints_[waypoint_idx_];
        const double &waypoint_y = waypoints_[waypoint_idx_ + 1];
        const double &rbt_x = rbt_pose_.pose.position.x;
        const double &rbt_y = rbt_pose_.pose.position.y;

        // if reached the waypoint, request for another waypoint
        double distance = std::hypot(waypoint_x - rbt_x, waypoint_y - rbt_y);
        if (distance < reached_thres_)
        {
            waypoint_idx_ += 2;
            // RCLCPP_INFO_STREAM(this->get_logger(),
            //                    "WAYPOINT " << (waypoint_idx_ / 2) // starts from 1.
            //                                << " (" << waypoint_x << "," << waypoint_y << ") REACHED!");
            cbTimer(); // recurse with new waypoint_idx_.
            return;
        }

        // request a plan
        geometry_msgs::msg::PoseStamped waypoint_pose;
        waypoint_pose.pose.position.x = waypoint_x;
        waypoint_pose.pose.position.y = waypoint_y;
        waypoint_pose.header.frame_id = rbt_pose_.header.frame_id;
        waypoint_pose.header.stamp = this->now();
        requestPlan(rbt_pose_, waypoint_pose);
    }

    void Behavior::requestPlan(const geometry_msgs::msg::PoseStamped &rbt_pose, const geometry_msgs::msg::PoseStamped &waypoint_pose)
    { // Send a non-blocking request. Doesn't care if a service request already exists
        if (plan_requested_)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "No request made as there is no response yet from previous request.");
            return;
        }
        plan_requested_ = true;
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        request->goal = waypoint_pose;
        request->start = rbt_pose;
        request_plan_future_ = client_plan_->async_send_request(request, std::bind(&Behavior::cbReceivePlan, this, std::placeholders::_1)).future;
        // RCLCPP_INFO_STREAM(this->get_logger(),
        //                    "Requested plan from Rbt("
        //                        << rbt_pose.pose.position.x << "," << rbt_pose.pose.position.y
        //                        << ") to Waypoint("
        //                        << waypoint_pose.pose.position.x << "," << waypoint_pose.pose.position.y << ")");
    }

    void Behavior::cbReceivePlan(const rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
    {
        (void)future;
        // RCLCPP_INFO_STREAM(this->get_logger(), "Plan received");
        // future.get()->plan;
        plan_requested_ = false;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee4308::turtle2::Behavior>());
    rclcpp::shutdown();
    return 0;
}