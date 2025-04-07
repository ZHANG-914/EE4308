#include <cmath>
#include "rclcpp/rclcpp.hpp"

#pragma once
namespace ee4308
{
    const double THRES = 1e-8; // for floating point calculations.

    // Finds the sign of a value
    template <typename T>
    T sgn(const T &value) { return (T(0) < value) - (value < T(0)); }

    // Constrains angles to -pi (inclusive) and pi (exclusive) radians.
    double limitAngle(const double &angle)
    {
        double result = std::fmod(angle + M_PI, M_PI * 2); // fmod rounds remainders to zero. we want remainders to be +ve like mod() in matlab and % in python
        return result >= 0 ? result - M_PI : result + M_PI;
    }

    // Finds the yaw from a quaternion.
    double getYawFromQuaternion(const double &qx, const double &qy, const double &qz, const double &qw)
    {
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // Finds the yaw from a quaternion-like object (must have `x`, `y`, `z`, `w` properties)
    template <typename T>
    double getYawFromQuaternion(const T &quaternion)
    {
        return ee4308::getYawFromQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    }

    // Calculates the quaternion values from yaw, assuming zero roll and pitch.
    void getQuaternionFromYaw(const double &yaw, double &qx, double &qy, double &qz, double &qw)
    {
        qw = std::cos(yaw / 2);
        qx = 0;
        qy = 0;
        qz = std::sin(yaw / 2);
    }

    // Calculates the quaternion values from yaw, assuming zero roll and pitch.
    template <typename T>
    void getQuaternionFromYaw(const double &yaw, T &quaternion)
    {
        ee4308::getQuaternionFromYaw(yaw, quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    }

    // Declares a parameter if it does not exist, and writes its value into `param_var`. The default value is written if the parameter is not set.
    template <typename U, typename T>
    void initParam(const U node_ptr, const std::string &param_name, T &param_var, const T &default_value)
    {
        if (node_ptr->has_parameter(param_name) == false)
            node_ptr->declare_parameter(param_name, default_value);

        node_ptr->get_parameter(param_name, param_var);
        RCLCPP_INFO_STREAM(node_ptr->get_logger(), param_name << ":\t" << param_var);
    }

    // Declares a double-vector parameter if it does not exist, and gets its value. The default value is written if the parameter is not set.
    template <typename U>
    void initParamDoubleArray(const U node_ptr, const std::string &param_name, std::vector<double> &param_var, const std::vector<double> &default_value)
    {
        if (node_ptr->has_parameter(param_name) == false)
            node_ptr->declare_parameter(param_name, default_value);

        param_var = node_ptr->get_parameter(param_name).as_double_array();

        std::stringstream ss;
        ss << param_name << ":\t[  ";
        for (const double &value : param_var)
            ss << value << ", ";
        ss << "\b\b" << "]";
        RCLCPP_INFO_STREAM(node_ptr->get_logger(), ss.str());
    }
}
