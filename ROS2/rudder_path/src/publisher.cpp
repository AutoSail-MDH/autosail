// Copyright 2016 Open Source Robotics Foundation, Inc.
// Modified by Erik Lindgren

#include <chrono>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Change these to your topics
#define PUB_TOPIC "/position/GOAL"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MSG std_msgs::msg::Float32MultiArray
#define STD_FLOAT std_msgs::msg::Float32

// The goal position to go towards
#define LAT 60.0
#define LON 16.0

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
   public:
    MinimalPublisher() : Node("goal_pub"), count_(0) {
        // Declare parameters
        this->declare_parameter<float>("lat", 0.0);
        this->declare_parameter<float>("long", 0.0);

        publisher_ = this->create_publisher<STD_MSG>(PUB_TOPIC, 10);
        // Change this timer to change how fast it publishes
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

   private:
    void timer_callback() {
        auto msg = STD_MSG();
        // get the current parameter values
        this->get_parameter("lat", g_lat);
        this->get_parameter("long", g_long);

        g_lat = abs(g_lat);
        g_long = abs(g_long);

        msg.data = {g_lat, g_long};
        rclcpp::sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(msg);
    }
    float g_lat;
    float g_long;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<STD_MSG>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
