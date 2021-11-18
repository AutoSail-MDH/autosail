
// Example written by Peter Nguyen, Modified by Erik Lindgren

#include <chrono>
#include <cmath>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Change these to your topics
#define SUB_TOPIC "/position/IMU"
#define PUB_TOPIC "/position/RUDDER_ANGLE"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MULTIFLOAT std_msgs::msg::Float32MultiArray
#define STD_FLOAT std_msgs::msg::Float32

using namespace std::chrono_literals;
using std::placeholders::_1;

float currAngle = 0;
int c = 0;
float earth_rad = 6371000;

class MinimalSubPub : public rclcpp::Node {
   public:
    MinimalSubPub() : Node("subpub") {
        subscriber_ = this->create_subscription<STD_MULTIFLOAT>(

            SUB_TOPIC, 50, std::bind(&MinimalSubPub::topic_callback, this, _1));

        publisher_ = this->create_publisher<STD_FLOAT>(PUB_TOPIC, 50);

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
    }

   private:
    void topic_callback(const STD_MULTIFLOAT::SharedPtr msg) {
        auto message = STD_FLOAT();
        // Sets the message that was read from the subscriber
        // float roll = msg->data[2];
        // float pitch = msg->data[1];
        float yaw = msg->data[0];

        float lat = 59.370687 * M_PIl / 180.0;
        float lon = 16.336636 * M_PIl / 180.0;
        float glat = 60.000000 * M_PIl / 180.0;
        float glon = 16.000000 * M_PIl / 180.0;

        float u = sin((glat - lat) / 2);
        float v = sin((glon - lon) / 2);
        float dDist = 2.0 * earth_rad * asin(sqrt(u * u + cos(lat) * cos(glat) * v * v));

        if (yaw < 0) {
            yaw = 180 + (180 + yaw);
        }

        currAngle = yaw;
        message.data = currAngle;
        printf("[%d] [Boat Angle: %.2f] [Distance to goal: %f]\n", c++, currAngle, dDist);
        currTime_ = nodeTime_->now();
        rclcpp::sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(message);
    }
    // Defines
    rclcpp::Subscription<STD_MULTIFLOAT>::SharedPtr subscriber_;
    rclcpp::Publisher<STD_FLOAT>::SharedPtr publisher_;
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
