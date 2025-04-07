#include "ee4308_drone/controller.hpp"

namespace ee4308::drone
{
    Controller::Controller(const std::string &name = "controller_ee4308") : Node(name)
    {
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "use_ground_truth", use_ground_truth_, false);
        initParam(this, "enable", enable_, true);
        initParam(this, "lookahead_distance", lookahead_distance_, 1.0);
        initParam(this, "max_xy_vel", max_xy_vel_, 1.0);
        initParam(this, "max_z_vel", max_z_vel_, 0.5);
        initParam(this, "yaw_vel", yaw_vel_, -0.3);
        initParam(this, "kp_xy", kp_xy_, 1.0);
        initParam(this, "kp_z", kp_z_, 1.0);

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            (use_ground_truth_ ? "odom" : "est_odom"), rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbOdom, this, std::placeholders::_1));
        sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbPlan, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1s / frequency_, std::bind(&Controller::cbTimer, this));
    }

    void Controller::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }

    void Controller::cbPlan(const nav_msgs::msg::Path msg)
    {
        plan_ = msg;
    }

    void Controller::cbTimer()
    {
        if (!enable_)
            return;

        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel(0, 0, 0, 0);
            return;
        }

        // ==== make use of ====
        // plan_.poses
        // odom_
        // ee4308::getYawFromQuaternion()
        // std::hypot()
        // std::clamp()
        // std::cos(), std::sin() 
        // lookahead_distance_
        // kp_xy_
        // kp_z_
        // max_xy_vel_
        // max_z_vel_
        // yaw_vel_
        // publishCmdVel()
        // ==== ====


        double x = odom_.pose.pose.position.x;
        double y = odom_.pose.pose.position.y;
        double z = odom_.pose.pose.position.z;
        RCLCPP_INFO(this->get_logger(), "Odom x: %.2f y: %.2f z: %.2f", odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);

        // === Get current yaw ===
        double yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);

        // === Find closest point on path ===
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        RCLCPP_INFO(this->get_logger(), "Plan size: %ld", plan_.poses.size());

        for (size_t i = 0; i < plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - x;
            double dy = plan_.poses[i].pose.position.y - y;
            double dz = plan_.poses[i].pose.position.z - z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // === Find lookahead point ===
        geometry_msgs::msg::PoseStamped lookahead_point = plan_.poses.back(); // default to last
        for (size_t i = closest_idx; i < plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - x;
            double dy = plan_.poses[i].pose.position.y - y;
            double dz = plan_.poses[i].pose.position.z - z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist >= lookahead_distance_)
            {
                lookahead_point = plan_.poses[i];
                break;
            }
        }

        // === Compute target vector in world frame ===
        double dx = lookahead_point.pose.position.x - x;
        double dy = lookahead_point.pose.position.y - y;
        double dz = lookahead_point.pose.position.z - z;

        // === Rotate to drone's frame ===
        double vx_world = dx;
        double vy_world = dy;

        double vx_body = std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
        double vy_body = -std::sin(yaw) * vx_world + std::cos(yaw) * vy_world;

        // === Proportional control with clamping ===
        double vx = std::clamp(kp_xy_ * vx_body, -max_xy_vel_, max_xy_vel_);
        double vy = std::clamp(kp_xy_ * vy_body, -max_xy_vel_, max_xy_vel_);
        double vz = std::clamp(kp_z_ * dz, -max_z_vel_, max_z_vel_);

        // === Publish cmd_vel with fixed yaw velocity ===
        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: %.2f %.2f %.2f %.2f", vx, vy, vz, yaw_vel_);
        publishCmdVel(vx, vy, vz, yaw_vel_);

    }

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        pub_cmd_vel_->publish(cmd_vel);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ee4308::drone::Controller>());
    rclcpp::shutdown();
    return 0;
}