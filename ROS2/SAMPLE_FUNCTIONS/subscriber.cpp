// Copyright 2016 Open Source Robotics Foundation, Inc.
// Modified by Erik Lindgren

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Change these to your topics
#define SUB_TOPIC "sample_topic_sub"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MSG std_msgs::msg::String

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
   public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        subscription_ =
            this->create_subscription<STD_MSG>(SUB_TOPIC, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

   private:
    void topic_callback(const STD_MSG::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<STD_MSG>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
