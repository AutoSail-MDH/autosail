// Copyright 2016 Open Source Robotics Foundation, Inc.
// Modified by Erik Lindgren

#include <chrono>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <autosail_message/msg/position_message.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Change these to your topics
#define PUB_TOPIC "/path/next_position"
// Change this to your message type, made this define to not have to write the long expression
#define POSITION_MSG autosail_message::msg::PositionMessage
//#define STD_FLOAT std_msgs::msg::Float32

// The goal position to go towards
#define LAT 60.0
#define LON 16.0

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SetNextPosition : public rclcpp::Node {
   public:
    SetNextPosition() : Node("rudder_control_node"), count_(0) {
        // Declare parameters
        this->declare_parameter<float>("lat", 0.0);
        this->declare_parameter<float>("long", 0.0);

        publisher_ = this->create_publisher<POSITION_MSG>(PUB_TOPIC, 10);
        // Change this timer to change how fast it publishes
        timer_ = this->create_wall_timer(500ms, std::bind(&SetNextPosition::position_callback, this));
    }

   private:
    void position_callback() {
        auto msg = POSITION_MSG();
        // get the current parameter values
        this->get_parameter("lat", goal_lat);
        this->get_parameter("long", goal_long);

        goal_lat = abs(goal_lat);
        goal_long = abs(goal_long);

        msg.goal_lat = goal_lat;
        msg.goal_long = goal_long;
        rclcpp::sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(msg);
    }
    float goal_lat;
    float goal_long;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<POSITION_MSG>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetNextPosition>());
    rclcpp::shutdown();
    return 0;
}
