
// Example written by Peter Nguyen, Modified by Erik Lindgren

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Change these to your topics
#define SUB_TOPIC "sample_topic_sub"
#define PUB_TOPIC "sample_topic_pub"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MSG std_msgs::msg::String

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubPub : public rclcpp::Node {
   public:
    MinimalSubPub() : Node("subpub") {
        subscriber_ = this->create_subscription<STD_MSG>(

            SUB_TOPIC, 50, std::bind(&MinimalSubPub::topic_callback, this, _1));  // No timer, instant response

        publisher_ = this->create_publisher<STD_MSG>(PUB_TOPIC, 50);

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
    }

   private:
    void topic_callback(const STD_MSG::SharedPtr msg) {
        auto message = STD_MSG();
        // Sets the message that was read from the subscriber
        message.data = msg->data;

        // Do your message processing here

        // Publishing of the message
        currTime_ = nodeTime_->now();
        rclcpp::sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(message);
    }
    // Defines
    rclcpp::Subscription<STD_MSG>::SharedPtr subscriber_;
    rclcpp::Publisher<STD_MSG>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time currTime_;
    rclcpp::Time prevTime_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubPub>());
    rclcpp::shutdown();
    return 0;
}
