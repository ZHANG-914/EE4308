#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"             // odom_drone
#include "geometry_msgs/msg/twist.hpp"           // gt_vel, cmd_vel
#include "geometry_msgs/msg/pose.hpp"            // gt_pose
#include "geometry_msgs/msg/point_stamped.hpp"   // altitude
#include "geometry_msgs/msg/vector3_stamped.hpp" // magnetic
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp" // gps
#include "sensor_msgs/msg/range.hpp"
#include "eigen3/Eigen/Dense"
#include "ee4308_core/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::drone
{
    class Estimator : public rclcpp::Node
    {
    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_est_odom_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sonar_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_baro_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_magnetic_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_; // ground_truth
        rclcpp::TimerBase::SharedPtr timer_;
        
        // States
        nav_msgs::msg::Odometry odom_; // ground truth
        Eigen::Vector2d Xx_;
        Eigen::Vector2d Xy_;
        Eigen::Vector2d Xz_;
        Eigen::Vector2d Xa_;
        Eigen::Matrix2d Px_;
        Eigen::Matrix2d Py_;
        Eigen::Matrix2d Pa_;
        Eigen::Matrix2d Pz_;
        Eigen::Vector3d initial_ECEF_;
        Eigen::Vector3d initial_position_;
        Eigen::Vector3d Ygps_;
        double Ymagnet_;
        double Ybaro_;
        double Ysonar_;
        double last_predict_time_;
        bool initialized_ecef_;
        bool initialized_magnetic_;

        // Parameters
        double frequency_;
        double var_imu_x_;
        double var_imu_y_;
        double var_imu_z_;
        double var_imu_a_;
        double var_gps_x_;
        double var_gps_y_;
        double var_gps_z_;
        double var_baro_;
        double var_sonar_;
        double var_magnet_;
        bool verbose_;

        // Constants
        static constexpr double GRAVITY = 9.8;
        static constexpr double RAD_POLAR = 6356752.3;
        static constexpr double RAD_EQUATOR = 6378137.0;

    public:
        /**
         * Constructor for the Estimator ROS Node.
         * @param name name of node.
         */
        explicit Estimator(const double &initial_x, const double &initial_y, const double &initial_z,
            const std::string &name);

    private:
        void cbTimer();

        Eigen::Vector3d getECEF(
            const double &sin_lat, const double &cos_lat,
            const double &sin_lon, const double &cos_lon,
            const double &alt);

        void cbGPS(const sensor_msgs::msg::NavSatFix msg);

        void cbSonar(const sensor_msgs::msg::Range msg);

        void cbMagnetic(const geometry_msgs::msg::Vector3Stamped msg);

        void cbBaro(const geometry_msgs::msg::PointStamped msg);

        void cbIMU(const sensor_msgs::msg::Imu msg);

        void cbOdom(const nav_msgs::msg::Odometry msg);
    };
}
