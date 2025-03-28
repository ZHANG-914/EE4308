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

        // !!! NOT ALLOWED TO USE ORIENTATION FROM IMU as ORIENTATION IS DERIVED FROM ANGULAR VELOCTIY !!!

        // !!! Store the states in Xx_, Xy_, Xz_, and Xa_.
        //      Store the covariances in Px_, Py_, Pz_, and Pa_.
        //      Required for terminal printing during demonstration.

        // ==== make use of ====
        // msg.linear_acceleration
        // msg.angular_velocity
        // GRAVITY
        // var_imu_x_, var_imu_y_, var_imu_z_, var_imu_a_
        // Xx_, Xy_, Xz_, Xa_
        // Px_, Py_, Pz_, Pa_
        // dt
        // std::cos(), std::sin()
        // ====  ====
        double az = msg.linear_acceleration.z - GRAVITY;

        Xz_(0) += Xz_(1) * dt + 0.5 * az * dt * dt;  
        Xz_(1) += az * dt;                            


        Eigen::Matrix2d Fz;
        Fz << 1, dt,
              0, 1;
        Eigen::Matrix2d Qz;
        Qz << var_imu_z_ * dt * dt, 0,
                                 0, var_imu_z_ * dt;
        // ==== [FOR LAB 2 ONLY] ==== 
        // for proj 2, comment out / delete the following, so the pink covariance bubble does not fill up RViz for lab 2 ====
        Px_ << 0.1, 0, 0, 0.1;
        Py_ << 0.1, 0, 0, 0.1;
        Pz_ = Fz * Pz_ * Fz.transpose() + Qz;
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
        Ysonar_ = msg.range;
        Eigen::Matrix<double, 1, 2> H;
        H << 1, 0;

        Eigen::Matrix<double, 1, 1> R;
        R(0, 0) = var_sonar_;

        double y = Ysonar_ - (H * Xz_)(0);

        Eigen::Matrix<double, 1, 1> S = H * Pz_ * H.transpose() + R;
        Eigen::Matrix<double, 2, 1> K = Pz_ * H.transpose() * S.inverse();
        Xz_ = Xz_ + K * y;

        Pz_ = (Eigen::Matrix2d::Identity() - K * H) * Pz_;
    }

    // ================================ GPS sub callback / EKF Correction ========================================
    Eigen::Vector3d Estimator::getECEF(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt)
    {
        Eigen::Vector3d ECEF;
        // ==== make use of ====
        // RAD_POLAR, RAD_EQUATOR
        // std::sqrt()
        // all the arguments above.
        // ====  ====

        return ECEF;
    }


    void Estimator::cbGPS(const sensor_msgs::msg::NavSatFix msg)
    {
        (void)msg;

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

        // ==== make use of ====
        // Ygps_
        // initial_position_
        // initial_ECEF_
        // sin_lat, cost_lat, sin_lon, cos_lon, alt
        // var_gps_x_, var_gps_y_, var_gps_z_
        // Px_, Py_, Pz_
        // Xx_, Xy_, Xz_
        //
        // - other Eigen methods like .transpose().
        // - Possible to divide a VectorXd element-wise by a double by using the divide operator '/'.
        // - Matrix multiplication using the times operator '*'.
        // ====  ====
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