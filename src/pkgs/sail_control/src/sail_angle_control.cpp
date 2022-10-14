
// Example written by Peter Nguyen, Modified by Erik Lindgren, Used and modified by Emma Jakobsson to fit wind_to_sail

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "autosail_message/msg/wind_message.hpp"
using namespace std;

// Change these to your topics
#define SUB_TOPIC "/sensor/wind"
#define PUB_TOPIC "/actuator/sail_angle"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MSG std_msgs::msg::Float32
#define WIND_MSG autosail_message::msg::WindMessage

using namespace std::chrono_literals;
using std::placeholders::_1;

class SailAngleControl : public rclcpp::Node {
   public:
    SailAngleControl() : Node("sail_angle_node") {
        subscriber_ = this->create_subscription<WIND_MSG>(
            SUB_TOPIC, 50, std::bind(&SailAngleControl::topic_callback, this, _1));  // No timer, instant response

        publisher_ = this->create_publisher<STD_MSG>(PUB_TOPIC, 50);

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
    }

   private:
    void topic_callback(const WIND_MSG::SharedPtr msg) {
        auto message = STD_MSG();

        // initialize variables
        float wind_angle = msg->wind_angle;
        float sail_angle = 0;
        int wind_direction = 0;

        // determine which wind direction the wind angle is
        if (wind_angle < 55 && wind_angle > 305) {
            wind_direction = 1;  // no go and close hauled zone
        } else if ((wind_angle >= 55 && wind_angle < 80) || (wind_angle > 280 && wind_angle <= 305)) {
            wind_direction = 2;  // close reach zone
        } else if ((wind_angle >= 80 && wind_angle < 120) || (wind_angle > 240 && wind_angle <= 280)) {
            wind_direction = 3;  // beam reach zone
        } else if ((wind_angle >= 120 && wind_angle < 160) || (wind_angle > 200 && wind_angle <= 240)) {
            wind_direction = 4;  // broad reach zone
        } else if (wind_angle >= 160 && wind_angle <= 200) {
            wind_direction = 5;  // running zone
        }

        // set angle according to wind direction
        switch (wind_direction) {
            case 1:  // no go and close hauled
                sail_angle = 0;
                break;
            case 2:  // close reach
                // determine which side of the boat the sail should point towards, 0-180 starboard 180-360 port
                if (wind_angle < 180) {
                    sail_angle = 30;
                } else {
                    sail_angle = -30;
                }
                break;
            case 3:  // beam reach
                // determine which side of the boat the sail should point towards, 0-180 starboard 180-360 port
                if (wind_angle < 180) {
                    sail_angle = 45;
                } else {
                    sail_angle = -45;
                }
                break;
            case 4:  // broad reach
                // determine which side of the boat the sail should point towards, 0-180 starboard 180-360 port
                if (wind_angle < 180) {
                    sail_angle = 60;
                } else {
                    sail_angle = -60;
                }
                break;
            case 5:  // running
                // determine which side of the boat the sail should point towards, 0-180 starboard 180-360 port
                if (wind_angle < 180) {
                    sail_angle = 90;
                } else {
                    sail_angle = -90;
                }
                break;
            default:
                break;
        }

        // set the message to publish to the angle of the sail
        message.data = sail_angle;
        printf("I heard: %d || Publishing: '%f'\n", msg->wind_angle, message.data);

        // Publishing of the message
        currTime_ = nodeTime_->now();
        rclcpp::sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(message);
    }
    // Defines
    rclcpp::Subscription<WIND_MSG>::SharedPtr subscriber_;
    rclcpp::Publisher<STD_MSG>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time currTime_;
    rclcpp::Time prevTime_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SailAngleControl>());
    rclcpp::shutdown();
    return 0;
}
