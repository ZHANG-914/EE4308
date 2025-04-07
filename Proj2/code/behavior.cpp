#include "ee4308_drone/behavior.hpp"

namespace ee4308::drone
{
    void Behavior::cbTimer()
    {
        // ==== make use of ====
        // state_
        // TAKEOFF, TURTLE_POSITION, TURTLE_WAYPOINT, INITIAL, LANDING, END
        // odom_
        // waypoint_x_, waypoint_y_, waypoint_z_
        // reached_thres_
        // ==== ====

        // if (1)  // change the 1: if (reached waypoint)
        // {
        //     if (state_ == TAKEOFF)
        //     {
        //         transition(INITIAL);
        //     }
        //     // ...
        // }

        // request a plan with requestPlan(). This is done every time cbTimer() is called.
        // ...
        double dx = waypoint_x_ - odom_.pose.pose.position.x;
        double dy = waypoint_y_ - odom_.pose.pose.position.y;
        double dz = waypoint_z_ - odom_.pose.pose.position.z;
        double distance = sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < reached_thres_) // 如果无人机到达目标点
        {
            if (state_ == TAKEOFF)
            {
                transition(INITIAL);
            }
            else if (state_ == INITIAL)
            {
                if (turtle_stop_)
                {
                    transition(LANDING);
                }
                else
                {
                    transition(TURTLE_POSITION);
                }
            }
            else if (state_ == TURTLE_POSITION)
            {
                transition(TURTLE_WAYPOINT);
            }
            else if (state_ == TURTLE_WAYPOINT)
            {
                transition(INITIAL);
            }
            else if (state_ == LANDING)
            {
                transition(END);
            }
        
        }
        RCLCPP_INFO(this->get_logger(), "State: %d", state_);

        
        requestPlan(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z,
                    waypoint_x_, waypoint_y_, waypoint_z_);
        
    }

    void Behavior::transition(int new_state)
    {
        // std::cout << "Transition from " << state_ << " To " << new_state << std::endl;

        // ==== make use of ====
        // new_state (function argument)
        // state_
        // TAKEOFF, TURTLE_POSITION, TURTLE_WAYPOINT, INITIAL, LANDING, END
        // initial_x_, initial_y_, initial_z_,
        // cruise_height_
        // turtle_plan_.poses
        // waypoint_x_, waypoint_y_, waypoint_z_
        // turtle_stop_
        // ==== ====

        // if (state_ == TAKEOFF)
        // {
        //     waypoint_x_ = initial_x_;
        //     waypoint_y_ = initial_y_;
        //     waypoint_z_ = cruise_height_;
        // }
        // else if (state_ == END)
        // {
        //     pub_land_->publish(std_msgs::msg::Empty()); // turn off the drone.
        //     timer_ = nullptr;                           // delete the timer to shutdown.
        // }
        RCLCPP_INFO_STREAM(this->get_logger(), "Transition from " << state_ << " to " << new_state);
        state_ = new_state;

        if (new_state == TAKEOFF)
        {
            waypoint_x_ = initial_x_;
            waypoint_y_ = initial_y_;
            waypoint_z_ = cruise_height_;
        }
        else if (new_state == INITIAL)
        {
            if (!turtle_plan_.poses.empty())
            {
                waypoint_x_ = initial_x_; 
                waypoint_y_ = initial_y_; 
                waypoint_z_ = cruise_height_;
            }
        }
        else if (new_state == TURTLE_POSITION)
        {
            if (!turtle_plan_.poses.empty())
            {
                waypoint_x_ = turtle_plan_.poses.front().pose.position.x;
                waypoint_y_ = turtle_plan_.poses.front().pose.position.y;
                waypoint_z_ = cruise_height_;
            }
        }
        else if (new_state == TURTLE_WAYPOINT)
        {
            if (!turtle_plan_.poses.empty())
            {
                waypoint_x_ = turtle_plan_.poses.back().pose.position.x;
                waypoint_y_ = turtle_plan_.poses.back().pose.position.y;
                waypoint_z_ = cruise_height_;
            }
        }
        else if (new_state == LANDING)
        {
            waypoint_x_ = initial_x_; // 降落到初始位置
            waypoint_y_ = initial_y_; // 降落到初始位置
            waypoint_z_ = initial_z_; // 降落到初始高度
        }
        else if (new_state == END)
        {
            pub_land_->publish(std_msgs::msg::Empty()); // 关闭无人机
            timer_ = nullptr; // 停止定时器
        }
    }

    Behavior::Behavior(
        const double initial_x, const double initial_y, const double initial_z,
        const std::string &name = "behavior")
        : Node(name), initial_x_(initial_x), initial_y_(initial_y), initial_z_(initial_z)
    {

        // parameters
        initParam(this, "reached_thres", reached_thres_, 0.3);
        initParam(this, "cruise_height", cruise_height_, 3.0);
        initParam(this, "frequency", frequency_, 5.0);
        initParam(this, "use_ground_truth", use_ground_truth_, false);

        // topics
        pub_takeoff_ = this->create_publisher<std_msgs::msg::Empty>("takeoff", rclcpp::ServicesQoS());
        pub_land_ = this->create_publisher<std_msgs::msg::Empty>("land", rclcpp::ServicesQoS());
        sub_turtle_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/turtle/stop", rclcpp::SensorDataQoS(),
            std::bind(&Behavior::cbTurtleStop, this, std::placeholders::_1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            (use_ground_truth_ ? "odom" : "est_odom"), rclcpp::SensorDataQoS(),
            std::bind(&Behavior::cbOdom, this, std::placeholders::_1));
        sub_turtle_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "/turtle/plan", rclcpp::SensorDataQoS(),
            std::bind(&Behavior::cbTurtlePlan, this, std::placeholders::_1));

        // services
        client_plan_ = this->create_client<nav_msgs::srv::GetPlan>("get_plan");

        // wait for messages
        rclcpp::Rate rate(2.0);
        while (rclcpp::ok() && (odom_.header.stamp.sec == 0)) // for proj 2 // || turtle_plan_.header.stamp.sec == 0))
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for subscribed topics");
            rate.sleep();
            rclcpp::spin_some(this->get_node_base_interface());
        }

        // states
        turtle_stop_ = false;
        plan_requested_ = false;
        state_ = BEGIN;
        waypoint_x_ = initial_x;
        waypoint_y_ = initial_y;
        waypoint_z_ = initial_z;
        pub_takeoff_->publish(std_msgs::msg::Empty()); // turn on the drone and start default takeoff routine
        transition(TAKEOFF);
        timer_ = this->create_wall_timer(1s / frequency_, std::bind(&Behavior::cbTimer, this));
    }

    void Behavior::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }

    void Behavior::cbTurtleStop(const std_msgs::msg::Empty msg)
    {
        (void)msg;
        turtle_stop_ = true;
    }

    void Behavior::cbTurtlePlan(const nav_msgs::msg::Path msg)
    {
        turtle_plan_ = msg;
    }

    void Behavior::requestPlan(double drone_x, double drone_y, double drone_z,
                               double waypoint_x, double waypoint_y, double waypoint_z)
    { 
        if (plan_requested_)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "No request made as there is no response yet from previous request.");
            return;
        }
        plan_requested_ = true;
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        request->goal.header.frame_id = "map";
        request->goal.header.stamp = this->now();
        request->goal.pose.position.x = waypoint_x;
        request->goal.pose.position.y = waypoint_y;
        request->goal.pose.position.z = waypoint_z;
        request->start.header.frame_id = "map";
        request->start.header.stamp = this->now();
        request->start.pose.position.x = drone_x;
        request->start.pose.position.y = drone_y;
        request->start.pose.position.z = drone_z;
        request_plan_future_ = client_plan_->async_send_request(request, std::bind(&Behavior::cbReceivePlan, this, std::placeholders::_1)).future;
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

    double initial_x = std::stod(argv[1]);
    double initial_y = std::stod(argv[2]);
    double initial_z = std::stod(argv[3]);

    rclcpp::spin(std::make_shared<ee4308::drone::Behavior>(initial_x, initial_y, initial_z));
    rclcpp::shutdown();
    return 0;
}
