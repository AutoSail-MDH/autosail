// Copyright 2016 Open Source Robotics Foundation, Inc.
// Modified by Erik Lindgren

#include <memory>
#include <std_msgs/msg/float32.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Change these to your topics
#define SUB_TOPIC "/position/RUDDER_ANGLE"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MSG std_msgs::msg::Float32

int c = 0;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
   public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        subscription_ =
            this->create_subscription<STD_MSG>(SUB_TOPIC, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

   private:
    void topic_callback(const STD_MSG::SharedPtr msg) const { printf("[%d] [Angle: %f]\n", c++, msg->data); }
    rclcpp::Subscription<STD_MSG>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
