#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ee4308_core/core.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::drone
{
    // struct TeleopParameters
    // {
    //     struct Topics
    //     {
    //         std::string cmd_vel = "cmd_vel";
    //         std::string takeoff = "takeoff";
    //         std::string land = "land";
    //     } topics;

    //     double linear_step = 0.1;
    //     double angular_step = 0.1;
    //     double linear_step_size = 0.01;
    //     double angular_step_size = 0.01;
    // };

    class Teleop : public rclcpp::Node
    {
    private:
        // Parameters
        double lin_vel_step_;
        double ang_vel_step_;
        double lin_vel_step_increment_;
        double ang_vel_step_increment_;

        // topics 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_; 
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;      
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;         

        rclcpp::TimerBase::SharedPtr timer_;              

        // other states / constants
        geometry_msgs::msg::Twist cmd_vel_;
        static constexpr double max_lin_vel_ = 1.0;  // from sjtu drone teleop
        static constexpr double max_ang_vel_ = 1.0; // from sjtu drone teleop

    public:
        explicit Teleop(const std::string &name = "teleop_drone") : Node(name)
        {
            initParam(this, "lin_vel_step", lin_vel_step_, 0.1);
            initParam(this, "ang_vel_step", ang_vel_step_, 0.1);
            initParam(this, "lin_vel_step_increment", lin_vel_step_increment_, 0.01);
            initParam(this, "ang_vel_step_increment", ang_vel_step_increment_, 0.01);

            // controls
            pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::ServicesQoS());
            pub_takeoff_ = this->create_publisher<std_msgs::msg::Empty>("takeoff", rclcpp::ServicesQoS());
            pub_land_ = this->create_publisher<std_msgs::msg::Empty>("land", rclcpp::ServicesQoS());

            // print to terminal
            std::cout << "=================== Controls ====================" << std::endl;
            std::cout << "1: Request takeoff" << std::endl; 
            std::cout << "2: Request land" << std::endl; 
            std::cout << "s: Stop / hover" << std::endl; 
            std::cout << "w:  Move forward: Increase x-axis linear velocity" << std::endl; 
            std::cout << "x: Move backward: Decrease x-axis linear velocity" << std::endl; 
            std::cout << "a:   Strafe left: Increase y-axis linear velocity" << std::endl; 
            std::cout << "d:  Strafe right: Decrease y-axis linear velocity" << std::endl; 
            std::cout << "q:      Yaw left: Increase z-axis angular velocity" << std::endl; 
            std::cout << "e:     Yaw right: Decrease z-axis angular velocity" << std::endl; 
            std::cout << "r:          Rise: Increase z-axis linear velocity" << std::endl; 
            std::cout << "f:          Fall: Decrease z-axis linear velocity" << std::endl; 
            std::cout << "W: Increase linear velocity increment" << std::endl; 
            std::cout << "X: Decrease linear velocity increment" << std::endl; 
            std::cout << "Q: Increase angular velocity increment" << std::endl; 
            std::cout << "E: Decrease angular velocity increment" << std::endl; 
            std::cout << "==================================================" << std::endl;


            timer_ = this->create_wall_timer(50ms, std::bind(&Teleop::cbTimer, this));
        }

    private:

        void cbTimer()
        {
            auto change = [](double &value, const double increment, const double limit) -> void { // limit is positive.
                value += increment;
                value = std::clamp(value, -limit, limit);
            };

            char c;

            if (getch(c) == true || rclcpp::ok() == false)
            {
                std::cout << std::endl;
                // std::cout << "Terminated / Exception" << std::endl;
                rclcpp::shutdown();
                return;
            }

            switch (c)
            {
            case 'w': // forward
                change(cmd_vel_.linear.x, lin_vel_step_, max_lin_vel_);
                break;

            case 's': // stop
                cmd_vel_ = geometry_msgs::msg::Twist();
                break;

            case 'a': // strafe left
                change(cmd_vel_.linear.y, lin_vel_step_, max_lin_vel_);
                break;

            case 'd': // strafe right
                change(cmd_vel_.linear.y, -lin_vel_step_, max_lin_vel_);
                break;

            case 'x': // reverse
                change(cmd_vel_.linear.x, -lin_vel_step_, max_lin_vel_);
                break;

            case 'q': // yaw left
                change(cmd_vel_.angular.z, ang_vel_step_, max_ang_vel_);
                break;

            case 'e': // yaw right
                change(cmd_vel_.angular.z, -ang_vel_step_, max_ang_vel_);
                break;

            case 'r': // rise
                change(cmd_vel_.linear.z, lin_vel_step_, max_lin_vel_);
                break;

            case 'f': // fall
                change(cmd_vel_.linear.z, -lin_vel_step_, max_lin_vel_);
                break;

            case '1': // takeoff
                std::cout << std::endl
                          << "Taking Off" << std::endl;
                pub_takeoff_->publish(std_msgs::msg::Empty());
                break;

            case '2': // land
                std::cout << std::endl
                          << "Landing" << std::endl;
                pub_land_->publish(std_msgs::msg::Empty());
                break;

            case 'W': // increase horizontal step
                change(lin_vel_step_, lin_vel_step_increment_, max_lin_vel_);
                break;

            case 'X': // decrease horizontal step
                change(lin_vel_step_, -lin_vel_step_increment_, lin_vel_step_increment_);
                break;

            case 'Q': // increase angular step
                change(ang_vel_step_, ang_vel_step_increment_, max_ang_vel_);
                break;

            case 'E': // decrease angular step
                change(ang_vel_step_, -ang_vel_step_increment_, ang_vel_step_increment_);
                break;

            default: // don't do anything.
                break;
            }
            std::cout << "\r[" << c << "]";
            std::cout << std::fixed;
            std::cout << " LinVel("
                      << std::setw(5) << std::setprecision(2) << cmd_vel_.linear.x << ", "
                      << std::setw(5) << std::setprecision(2) << cmd_vel_.linear.y << ", "
                      << std::setw(5) << std::setprecision(2) << cmd_vel_.linear.z << ")";
            std::cout << " YawVel("
                      << std::setw(5) << std::setprecision(2) << cmd_vel_.angular.z << ")";
            std::cout << " Steps(Lin:"
                      << std::setw(5) << std::setprecision(2) << lin_vel_step_ << ", Ang"
                      << std::setw(5) << std::setprecision(2) << ang_vel_step_ << ")";

            std::cout << "    "; // pad some spaces just in case
            std::cout.flush();
            std::cout << "\b\b\b\b"; // remove extra nonsense characters.
            std::cout.flush();

            pub_cmd_vel_->publish(cmd_vel_);
        }

        bool getch(char &c)
        {
            c = 0;
            termios old;
            bool error = false;
            if (tcgetattr(0, &old) < 0)
                return true; // perror("tcsetattr()");
            old.c_lflag &= ~ICANON;
            old.c_lflag &= ~ECHO;
            old.c_cc[VMIN] = 1;
            old.c_cc[VTIME] = 0;
            if (tcsetattr(0, TCSANOW, &old) < 0)
                error = true; // perror("tcsetattr ICANON");
            if (read(0, &c, 1) < 0)
                error = true;
            old.c_lflag |= ICANON;
            old.c_lflag |= ECHO;
            if (tcsetattr(0, TCSADRAIN, &old) < 0)
                error = true; // perror("tcsetattr ~ICANON");
            return error;
        }
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor exe;

    // mapper node
    auto node = std::make_shared<ee4308::drone::Teleop>(
        "teleop" // node name
    );

    exe.add_node(node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
