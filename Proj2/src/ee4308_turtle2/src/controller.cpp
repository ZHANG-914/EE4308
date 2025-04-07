#include "ee4308_turtle2/controller.hpp"

namespace ee4308::turtle2
{
    Controller::Controller(const std::string &name = "controller") : Node(name)
    {
        // parameters
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "kp_lin", kp_lin_, 1.0);
        initParam(this, "kp_ang", kp_ang_, 1.0);
        initParam(this, "max_lin_vel", max_lin_vel_, 0.2);
        initParam(this, "max_ang_vel", max_ang_vel_, 1.0);
        initParam(this, "xy_tolerance", xy_tolerance_, 0.05);
        initParam(this, "yaw_tolerance", yaw_tolerance_, M_PI / 4);
        initParam(this, "lookahead_distance", lookahead_distance_, 0.3);
        initParam(this, "enable", enable_, true);

        // topics
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbOdom, this, std::placeholders::_1));
        sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbPlan, this, std::placeholders::_1));
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());

        // wait for topics to publish
        rclcpp::Rate rate(2.0);
        while (rclcpp::ok() && (rbt_pose_.header.stamp.sec == 0))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for subscribed topics");
            rate.sleep();
        }
        
        // Initialize main loop
        timer_ = this->create_wall_timer(1s / frequency_,
                                         std::bind(&Controller::cbTimer, this));

    }

    void Controller::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        rbt_pose_.pose = msg.pose.pose;
        rbt_pose_.header = msg.header;
    }

    void Controller::cbPlan(const nav_msgs::msg::Path msg)
    {
        plan_ = msg;
    }

    void Controller::cbTimer()
    {
        if (!enable_)
            return;

        double rbt_x = rbt_pose_.pose.position.x;
        double rbt_y = rbt_pose_.pose.position.y;
        double rbt_yaw = ee4308::getYawFromQuaternion(rbt_pose_.pose.orientation);

        if (plan_.poses.empty())
        {
            publishCmdVel(0, 0);
            return; // no plan.
        }

        // finds first point from start of path that satisfies lookahead. Uses goal pose if none can be found. assumes robot is slow enough.
        auto exceedsLookahead = [&](const geometry_msgs::msg::PoseStamped &plan_pose)
        { return std::hypot(plan_pose.pose.position.x - rbt_x, plan_pose.pose.position.y - rbt_y) > lookahead_distance_; };
        auto it_plan_poses = std::find_if(plan_.poses.begin(), plan_.poses.end(), exceedsLookahead);
        const geometry_msgs::msg::PoseStamped &lookahead_pose = (it_plan_poses == plan_.poses.end()) ? *std::prev(plan_.poses.end()) : *it_plan_poses;
        double dx = lookahead_pose.pose.position.x - rbt_x;
        double dy = lookahead_pose.pose.position.y - rbt_y;

        double err_ang = limitAngle(std::atan2(dy, dx) - rbt_yaw);
        double err_lin = std::hypot(dx, dy);

        double lin_vel = kp_lin_ * err_lin;
        double ang_vel = kp_ang_ * err_ang;

        if (err_lin < xy_tolerance_)
        {
            ang_vel = 0;
            lin_vel = 0;
        }

        if (std::abs(err_ang) < yaw_tolerance_)
            lin_vel *= (1 - std::abs(err_ang) / yaw_tolerance_);
        else
            lin_vel = 0;

        lin_vel = std::clamp(lin_vel, -max_lin_vel_, max_lin_vel_);
        ang_vel = std::clamp(ang_vel, -max_ang_vel_, max_ang_vel_);

        publishCmdVel(lin_vel, ang_vel);
    }

    void Controller::publishCmdVel(const double &lin_vel, const double &ang_vel)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = lin_vel;
        msg.angular.z = ang_vel;
        pub_cmd_vel_->publish(msg);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee4308::turtle2::Controller>());
    rclcpp::shutdown();
    return 0;
}