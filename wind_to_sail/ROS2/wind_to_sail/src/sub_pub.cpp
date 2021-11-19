
// Example written by Peter Nguyen, Modified by Erik Lindgren, Used and modified by Emma Jakobsson to fit wind_to_sail

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "sail.hpp"
#include <iostream>

#include "std_msgs/msg/float32.hpp"
using namespace std;

// Change these to your topics
#define SUB_TOPIC "hejhej"  //"/direction/wind"
#define PUB_TOPIC "/sail/angle"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MSG std_msgs::msg::Float32

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
        // message.data = msg->data;

        // Do your message processing here
        float wind_angle = msg->data;
        float sail_angle = 0;
        int wind_direction = 0;

        // determine which wind direction the wind angle is
        // wind_direction = get_direction(wind_angle);
        if (wind_angle < 55 && wind_angle > 305) {
            wind_direction = 1;
        } else if ((wind_angle >= 55 && wind_angle < 80) || (wind_angle > 280 && wind_angle <= 305)) {
            wind_direction = 2;
        } else if ((wind_angle >= 80 && wind_angle < 120) || (wind_angle > 240 && wind_angle <= 280)) {
            wind_direction = 3;
        } else if ((wind_angle >= 120 && wind_angle < 160) || (wind_angle > 200 && wind_angle <= 240)) {
            wind_direction = 4;
        } else if (wind_angle >= 160 && wind_angle <= 200) {
            wind_direction = 5;
        }

        // set angle according to wind direction
        // sail_angle = set_angle(wind_direction, wind_angle);
        switch (wind_direction) {
            case 1:  // no go and close hauled
                sail_angle = 0;
                break;
            case 2:  // close reach
                if (wind_angle < 180) {
                    sail_angle = 30;
                } else {
                    sail_angle = -30;
                }
                break;
            case 3:  // beam reach
                if (wind_angle < 180) {
                    sail_angle = 45;
                } else {
                    sail_angle = -45;
                }
                break;
            case 4:  // broad reach
                if (wind_angle < 180) {
                    sail_angle = 60;
                } else {
                    sail_angle = -60;
                }
                break;
            case 5:  // running
                if (wind_angle < 180) {
                    sail_angle = 90;
                } else {
                    sail_angle = -90;
                }
                break;
            default:
                break;
        }

        // set the message to publish to the angle to set the sail
        message.data = sail_angle;
        printf("I heard: %f || Publishing: '%f'\n", msg->data, message.data);

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
