#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::cleanup() { RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

    // ====================================== LAB 1, PROJ 1 ====================================================

    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        tf_ = tf;
        plugin_name_ = name;

        // initialize parameters
        initParam(node_, plugin_name_ + ".desired_linear_vel", desired_linear_vel_, 0.2);
        initParam(node_, plugin_name_ + ".desired_lookahead_dist", desired_lookahead_dist_, 0.4);
        initParam(node_, plugin_name_ + ".max_angular_vel", max_angular_vel_, 1.0);
        initParam(node_, plugin_name_ + ".max_linear_vel", max_linear_vel_, 0.22);
        initParam(node_, plugin_name_ + ".xy_goal_thres", xy_goal_thres_, 0.05);
        initParam(node_, plugin_name_ + ".yaw_goal_thres", yaw_goal_thres_, 0.25);

        // initialize topics
        // sub_scan_ = node_->create_subscription<some msg type>(
        //     "some topic", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::some_callback, this, std::placeholders::_1));
    }



    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
        double dx = goal_pose.pose.position.x - pose.pose.position.x;
        double dy = goal_pose.pose.position.y - pose.pose.position.y;
        double distance_to_goal = std::hypot(dx, dy);

        if (distance_to_goal < xy_goal_thres_) {
            return writeCmdVel(0, 0);
        }

        // Find the closest point on the path
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < global_plan_.poses.size(); i++) {
            double dist = std::hypot(global_plan_.poses[i].pose.position.x - pose.pose.position.x,
                                    global_plan_.poses[i].pose.position.y - pose.pose.position.y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // Find the lookahead point
        size_t lookahead_idx = closest_idx;
        double lookahead_dist = 0.0;

        while (lookahead_idx < global_plan_.poses.size() && lookahead_dist < desired_lookahead_dist_) {
            double dx = global_plan_.poses[lookahead_idx].pose.position.x - pose.pose.position.x;
            double dy = global_plan_.poses[lookahead_idx].pose.position.y - pose.pose.position.y;
            lookahead_dist = std::hypot(dx, dy);
            lookahead_idx++;
        }

        if (lookahead_idx >= global_plan_.poses.size()) {
            lookahead_idx = global_plan_.poses.size() - 1;
        }


        // get lookahead
        geometry_msgs::msg::PoseStamped lookahead_pose = global_plan_.poses[lookahead_idx];

        // double linear_vel = 0 * (lookahead_pose.pose.position.x - pose.pose.position.x);
        // double angular_vel = 0 * getYawFromQuaternion(goal_pose.pose.orientation);

        // Transform the lookahead point to the robot frame
        tf2::Transform tf_map_to_base;
        tf2::fromMsg(pose.pose, tf_map_to_base);
        tf2::Vector3 lookahead_world(lookahead_pose.pose.position.x, lookahead_pose.pose.position.y, 0);
        tf2::Vector3 lookahead_robot = tf_map_to_base.inverse() * lookahead_world;

        // double x_prime = lookahead_robot.x();
        double y_prime = lookahead_robot.y();

        // Calculate curvature
        double curvature = (2 * y_prime) / (desired_lookahead_dist_ * desired_lookahead_dist_);

        // Calculate angular velocity
        double angular_vel = desired_linear_vel_ * curvature;

        // Constrain angular velocity
        if (angular_vel > max_angular_vel_) {
            angular_vel = max_angular_vel_;
        } else if (angular_vel < -max_angular_vel_) {
            angular_vel = -max_angular_vel_;
        }

        // Constrain linear velocity
        double linear_vel = desired_linear_vel_;
        if (linear_vel > max_linear_vel_) {
            linear_vel = max_linear_vel_;
        }

        return writeCmdVel(linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)