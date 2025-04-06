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
        // Get current position
        auto &pos = odom_.pose.pose.position;
        double x = pos.x, y = pos.y, z = pos.z;

        // Get current yaw
        double yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);

        // Step 1: Find closest point on the path
        int closest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = 0; i < plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - x;
            double dy = plan_.poses[i].pose.position.y - y;
            double dist = dx * dx + dy * dy;
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // Step 2: Find lookahead point
        geometry_msgs::msg::PoseStamped target = plan_.poses.back(); // default to last
        for (int i = closest_idx; i < plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - x;
            double dy = plan_.poses[i].pose.position.y - y;
            double dist = std::hypot(dx, dy);
            if (dist >= lookahead_distance_)
            {
                target = plan_.poses[i];
                break;
            }
        }

        // Step 3: Compute direction vector to lookahead point (in world frame)
        double dx_world = target.pose.position.x - x;
        double dy_world = target.pose.position.y - y;
        double dz = target.pose.position.z - z;

        // Convert world frame dx/dy to body frame
        double dx_body = std::cos(-yaw) * dx_world - std::sin(-yaw) * dy_world;
        double dy_body = std::sin(-yaw) * dx_world + std::cos(-yaw) * dy_world;

        // Apply proportional control (can tweak kp_xy_ and kp_z_)
        double vx = kp_xy_ * dx_body;
        double vy = kp_xy_ * dy_body;
        double vz = kp_z_ * dz;

        // Clamp velocities
        double scale = std::hypot(vx, vy);
        if (scale > max_xy_vel_)
        {
            double ratio = max_xy_vel_ / scale;
            vx *= ratio;
            vy *= ratio;
        }

        vz = std::clamp(vz, -max_z_vel_, max_z_vel_);

        // Publish command velocity with constant yaw velocity
        publishCmdVel(vx, vy, vz, yaw_vel_);





        // publish
        // publishCmdVel(0, 0, 0, 0);
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