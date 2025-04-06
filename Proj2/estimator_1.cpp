#include "ee4308_drone/estimator.hpp"

namespace ee4308::drone
{
    // ================================ IMU sub callback / EKF Prediction ========================================
    void Estimator::cbIMU(const sensor_msgs::msg::Imu msg)
    {
        rclcpp::Time tnow = msg.header.stamp;
        double dt = tnow.seconds() - last_predict_time_;
        last_predict_time_ = tnow.seconds();

        if (dt < ee4308::THRES)
            return;

        // EKF Prediction step for x position and velocity
        Eigen::Matrix2d F = Eigen::Matrix2d::Identity();
        F(0, 1) = dt;
        
        // Process noise covariance matrix for x
        Eigen::Matrix2d Q_x = Eigen::Matrix2d::Zero();
        Q_x(1, 1) = var_imu_x_ * dt;
        
        // State transition for x
        Xx_ = F * Xx_;
        // Add control input: acceleration in the x direction considering rotation
        double cos_yaw = std::cos(Xa_(0));
        double sin_yaw = std::sin(Xa_(0));
        Xx_(1) += (msg.linear_acceleration.x * cos_yaw - msg.linear_acceleration.y * sin_yaw) * dt;
        
        // Update covariance for x
        Px_ = F * Px_ * F.transpose() + Q_x;
        
        // EKF Prediction step for y position and velocity
        Eigen::Matrix2d F_y = Eigen::Matrix2d::Identity();
        F_y(0, 1) = dt;
        
        // Process noise covariance matrix for y
        Eigen::Matrix2d Q_y = Eigen::Matrix2d::Zero();
        Q_y(1, 1) = var_imu_y_ * dt;
        
        // State transition for y
        Xy_ = F_y * Xy_;
        // Add control input: acceleration in the y direction considering rotation
        Xy_(1) += (msg.linear_acceleration.x * sin_yaw + msg.linear_acceleration.y * cos_yaw) * dt;
        
        // Update covariance for y
        Py_ = F_y * Py_ * F_y.transpose() + Q_y;
        
        // EKF Prediction step for z position and velocity
        Eigen::Matrix2d F_z = Eigen::Matrix2d::Identity();
        F_z(0, 1) = dt;
        
        // Process noise covariance matrix for z
        Eigen::Matrix2d Q_z = Eigen::Matrix2d::Zero();
        Q_z(1, 1) = var_imu_z_ * dt;
        
        // State transition for z
        Xz_ = F_z * Xz_;
        // Add control input: acceleration in the z direction (considering gravity)
        Xz_(1) += (msg.linear_acceleration.z + GRAVITY) * dt;
        
        // Update covariance for z
        Pz_ = F_z * Pz_ * F_z.transpose() + Q_z;
        
        // EKF Prediction step for yaw angle and angular velocity
        Eigen::Matrix2d F_a = Eigen::Matrix2d::Identity();
        F_a(0, 1) = dt;
        
        // Process noise covariance matrix for yaw
        Eigen::Matrix2d Q_a = Eigen::Matrix2d::Zero();
        Q_a(1, 1) = var_imu_a_ * dt;
        
        // State transition for yaw
        Xa_ = F_a * Xa_;
        // Add control input: angular velocity around z axis
        Xa_(1) = msg.angular_velocity.z;
        // Update yaw angle
        Xa_(0) += Xa_(1) * dt;
        // Limit angle to [-π, π]
        Xa_(0) = limitAngle(Xa_(0));
        
        // Update covariance for yaw
        Pa_ = F_a * Pa_ * F_a.transpose() + Q_a;

        // ==== [FOR LAB 2 ONLY] ==== 
        // for proj 2, comment out / delete the following, so the pink covariance bubble does not fill up RViz for lab 2 ====
        // Px_ << 0.1, 0, 0, 0.1;
        // Py_ << 0.1, 0, 0, 0.1;
        // ==== ====
    }

    // ================================ Sonar sub callback / EKF Correction ========================================
    void Estimator::cbSonar(const sensor_msgs::msg::Range msg)
    {
        (void)msg;

        if (msg.range > msg.max_range)
        { // skip erroneous measurements
            return;
        }

        // ==== make use of ====
        // msg.range
        // Ysonar_
        // var_sonar_
        // Xz_
        // Pz_
        // .transpose()
        // ====  ====
    }

    // ================================ GPS sub callback / EKF Correction ========================================
    Eigen::Vector3d Estimator::getECEF(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt)
    {
        Eigen::Vector3d ECEF;
        
        // Earth is an ellipsoid, not a sphere
        // First calculate the radius at the given latitude
        double N = RAD_EQUATOR / std::sqrt(1.0 - (1.0 - std::pow(RAD_POLAR / RAD_EQUATOR, 2)) * std::pow(cos_lat, 2));
        
        // Calculate ECEF coordinates
        ECEF(0) = (N + alt) * cos_lat * cos_lon; // X coordinate
        ECEF(1) = (N + alt) * cos_lat * sin_lon; // Y coordinate
        ECEF(2) = (N * std::pow(RAD_POLAR / RAD_EQUATOR, 2) + alt) * sin_lat; // Z coordinate
        
        return ECEF;
    }


    void Estimator::cbGPS(const sensor_msgs::msg::NavSatFix msg)
    {
        constexpr double DEG2RAD = M_PI / 180;
        double lat = -msg.latitude * DEG2RAD;  // !!! Gazebo spherical coordinates have a bug. Need to negate.
        double lon = -msg.longitude * DEG2RAD; // !!! Gazebo spherical coordinates have a bug. Need to negate.
        double alt = msg.altitude;

        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        if (initialized_ecef_ == false)
        {
            initial_ECEF_ = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);
            initialized_ecef_ = true;
            return;
        }

        Eigen::Vector3d ECEF = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);
        
        // Calculate the local position based on ECEF coordinates
        // Define rotation matrix from ECEF to local frame
        Eigen::Matrix3d R_ECEF_to_local;
        R_ECEF_to_local << -sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon,
                           cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon,
                           0, cos_lat, sin_lat;
        
        // Calculate local position (ENU - East, North, Up)
        Ygps_ = R_ECEF_to_local * (ECEF - initial_ECEF_) + initial_position_;
        
        // EKF Update step for x position
        Eigen::Matrix<double, 1, 2> H_x;
        H_x << 1, 0; // We're measuring position directly
        
        // Kalman gain for x
        double R_x = var_gps_x_;
        Eigen::Vector2d K_x = Px_ * H_x.transpose() / (H_x * Px_ * H_x.transpose() + R_x);
        
        // Update state estimate for x
        Xx_ = Xx_ + K_x * (Ygps_(0) - H_x * Xx_);
        
        // Update covariance for x
        Px_ = (Eigen::Matrix2d::Identity() - K_x * H_x) * Px_;
        
        // EKF Update step for y position
        Eigen::Matrix<double, 1, 2> H_y;
        H_y << 1, 0; // We're measuring position directly
        
        // Kalman gain for y
        double R_y = var_gps_y_;
        Eigen::Vector2d K_y = Py_ * H_y.transpose() / (H_y * Py_ * H_y.transpose() + R_y);
        
        // Update state estimate for y
        Xy_ = Xy_ + K_y * (Ygps_(1) - H_y * Xy_);
        
        // Update covariance for y
        Py_ = (Eigen::Matrix2d::Identity() - K_y * H_y) * Py_;
        
        // EKF Update step for z position
        Eigen::Matrix<double, 1, 2> H_z;
        H_z << 1, 0; // We're measuring position directly
        
        // Kalman gain for z
        double R_z = var_gps_z_;
        Eigen::Vector2d K_z = Pz_ * H_z.transpose() / (H_z * Pz_ * H_z.transpose() + R_z);
        
        // Update state estimate for z
        Xz_ = Xz_ + K_z * (Ygps_(2) - H_z * Xz_);
        
        // Update covariance for z
        Pz_ = (Eigen::Matrix2d::Identity() - K_z * H_z) * Pz_;
    }

    // ================================ Magnetic sub callback / EKF Correction ========================================
    void Estimator::cbMagnetic(const geometry_msgs::msg::Vector3Stamped msg)
    {
        (void)msg;
        // Along the horizontal plane, the magnetic north in Gazebo points towards +x, when it should point to +y. It is a bug.
        // As the drone always starts pointing towards +x, there is no need to offset the calculation with an initial heading.
        
        // magnetic force direction in drone's z-axis can be ignored.

        // ==== make use of ====
        // Ymagnet_
        // msg.vector.x // the magnetic force direction along drone's x-axis.
        // msg.vector.y // the magnetic force direction along drone's y-axis.
        // std::atan2()
        // Xa_
        // Pa_
        // var_magnet_
        // .transpose()
        // limitAngle()
        // ====  ====
    }

    // ================================ Baro sub callback / EKF Correction ========================================
    void Estimator::cbBaro(const geometry_msgs::msg::PointStamped msg)
    {
        // if this section is done, Pz_ has to be augmented with the barometer bias to become a 3-state vector.

        (void)msg;

        // ==== make use of ====
        // Ybaro_ 
        // msg.point.z
        // var_baro_
        // Pz_
        // Xz_
        // .transpose()
        // ====  ====
    }


    Estimator::Estimator(
        const double &initial_x, const double &initial_y, const double &initial_z,
        const std::string &name = "estimator")
        : Node(name)
    {
        // parameters
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "var_imu_x", var_imu_x_, 0.2);
        initParam(this, "var_imu_y", var_imu_y_, 0.2);
        initParam(this, "var_imu_z", var_imu_z_, 0.2);
        initParam(this, "var_imu_a", var_imu_a_, 0.2);
        initParam(this, "var_gps_x", var_gps_x_, 0.2);
        initParam(this, "var_gps_y", var_gps_y_, 0.2);
        initParam(this, "var_gps_z", var_gps_z_, 0.2);
        initParam(this, "var_baro", var_baro_, 0.2);
        initParam(this, "var_sonar", var_sonar_, 0.2);
        initParam(this, "var_magnet", var_magnet_, 0.2);
        initParam(this, "verbose", verbose_, true);

        // topics
        pub_est_odom_ = create_publisher<nav_msgs::msg::Odometry>("est_odom", rclcpp::ServicesQoS());
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos, std::bind(&Estimator::cbOdom, this, std::placeholders::_1)); // ground truth in sim.
        sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", qos, std::bind(&Estimator::cbGPS, this, std::placeholders::_1));
        sub_sonar_ = create_subscription<sensor_msgs::msg::Range>(
            "sonar", qos, std::bind(&Estimator::cbSonar, this, std::placeholders::_1));
        sub_magnetic_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "magnetic", qos, std::bind(&Estimator::cbMagnetic, this, std::placeholders::_1));
        sub_baro_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "altitude", qos, std::bind(&Estimator::cbBaro, this, std::placeholders::_1));
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&Estimator::cbIMU, this, std::placeholders::_1));

        // states
        initial_position_ << initial_x, initial_y, initial_z;
        Xx_ << initial_x, 0;
        Xy_ << initial_y, 0;
        Xz_ << initial_z, 0; // , 0 // change hpp as well.
        Xa_ << 0, 0;
        Px_ = Eigen::Matrix2d::Constant(1e3),
        Py_ = Eigen::Matrix2d::Constant(1e3),
        Pz_ = Eigen::Matrix2d::Constant(1e3); // Matrix3d; change hpp as well.
        Pa_ = Eigen::Matrix2d::Constant(1e3);
        initial_ECEF_ << NAN, NAN, NAN;
        Ygps_ << NAN, NAN, NAN;
        Ymagnet_ = NAN;
        Ybaro_ = NAN;
        Ysonar_ = NAN;

        last_predict_time_ = this->now().seconds();
        initialized_ecef_ = false;
        initialized_magnetic_ = false;

        timer_ = this->create_wall_timer(
            1s / frequency_,
            std::bind(&Estimator::cbTimer, this));
    }

    void Estimator::cbTimer()
    {
        nav_msgs::msg::Odometry est_odom;

        est_odom.header.stamp = this->now();
        est_odom.child_frame_id = "";     //; std::string(this->get_namespace()) + "/base_footprint";
        est_odom.header.frame_id = "map"; //; std::string(this->get_namespace()) + "/odom";

        est_odom.pose.pose.position.x = Xx_[0];
        est_odom.pose.pose.position.y = Xy_[0];
        est_odom.pose.pose.position.z = Xz_[0];
        getQuaternionFromYaw(Xa_[0], est_odom.pose.pose.orientation);
        est_odom.pose.covariance[0] = Px_(0, 0);
        est_odom.pose.covariance[7] = Py_(0, 0);
        est_odom.pose.covariance[14] = Pz_(0, 0);
        est_odom.pose.covariance[35] = Pa_(0, 0);

        est_odom.twist.twist.linear.x = Xx_[1];
        est_odom.twist.twist.linear.y = Xy_[1];
        est_odom.twist.twist.linear.z = Xz_[1];
        est_odom.twist.twist.angular.z = Xa_[1];
        est_odom.twist.covariance[0] = Px_(1, 1);
        est_odom.twist.covariance[7] = Py_(1, 1);
        est_odom.twist.covariance[14] = Pz_(1, 1);
        est_odom.twist.covariance[35] = Pa_(1, 1);

        pub_est_odom_->publish(est_odom);

        if (verbose_)
        {

            RCLCPP_INFO_STREAM(this->get_logger(), "===");
            std::cout << std::fixed;
            std::cout << "    Pose("
                      << std::setw(7) << std::setprecision(3) << Xx_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(0) << ","
                      << std::setw(7) << std::setprecision(3) << limitAngle(Xa_(0)) << ")"
                      << std::endl;
            std::cout << "   Twist("
                      << std::setw(7) << std::setprecision(3) << Xx_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xa_(1) << ")"
                      << std::endl;
            std::cout << " ErrPose("
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.x - Xx_(0) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.y - Xy_(0) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.z - Xz_(0) << ","
                      << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(ee4308::getYawFromQuaternion(odom_.pose.pose.orientation) - Xa_(0)) << ")"
                      << std::endl;
            std::cout << "ErrTwist("
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.x - Xx_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.y - Xy_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.z - Xz_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.angular.z - Xa_(1) << ")"
                      << std::endl;
            std::cout << "     GPS("
                      << std::setw(7) << std::setprecision(3) << Ygps_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(2) << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            // std::cout << "    Baro("
            //           << std::setw(8) << "---  ,"
            //           << std::setw(8) << "---  ,"
            //           << std::setw(7) << std::setprecision(3) << Ybaro_ << ","
            //           << std::setw(8) << "---  )"
            //           << std::endl;
            // std::cout << "   BBias("
            //           << std::setw(8) << "---  ,"
            //           << std::setw(8) << "---  ,"
            //           << std::setw(7) << std::setprecision(3) << Xz_(2) << ","
            //           << std::setw(8) << "---  )"
            //           << std::endl;
            std::cout << "   Sonar("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ysonar_ << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "   Magnt("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ymagnet_ << ")"
                      << std::endl;
        }
    }

    void Estimator::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    double initial_x = std::stod(argv[1]);
    double initial_y = std::stod(argv[2]);
    double initial_z = std::stod(argv[3]);

    rclcpp::spin(std::make_shared<ee4308::drone::Estimator>(initial_x, initial_y, initial_z));
    rclcpp::shutdown();
    return 0;
}