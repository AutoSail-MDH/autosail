// Standard C++ headers
#include <math.h>

#include <chrono>
#include <cstring>
#include <memory>

// Dependencies
#include <autosail_message/msg/velocity_message.hpp>
#include <autosail_message/msg/gnss_message.hpp>

#include "rclcpp/rclcpp.hpp"  //Common for ROS 2

using namespace std::chrono_literals;
using std::placeholders::_1;


#define VELOCITY_MSG autosail_message::msg::VelocityMessage
#define GNSS_MSG autosail_message::msg::GNSSMessage

class VelocityCalculation : public rclcpp::Node  // Create node class by inheriting
{                                          // this refers to the MinimalPublisher node
   public:
    VelocityCalculation()  // The public constructor intits node
        : Node("velocity_node") {
        subscriber_pose_ =
            this->create_subscription<GNSS_MSG>(  // Constructor uses the node's
                                                                          // create_subscription class for callbacks
                "/sensor/gnss", 50, // "/sensor/gnss"
                std::bind(&VelocityCalculation::gnss_topic_callback, this, _1));  // No timer, instant response

        publisher_ = this->create_publisher<VELOCITY_MSG>(
            "/position/velocity", 50);  // Init msg type, topic name and msg size

        node_clock_ = this->get_clock();  // Create clock starting at the time of node creation
    }

   private:
    void gnss_topic_callback(const GNSS_MSG::SharedPtr msg) {
        auto message = autosail_message::msg::VelocityMessage();

        float current_gnss_[2] = {0.0, 0.0};
        current_gnss_[0] = msg->position.latitude;
        current_gnss_[1] = msg->position.longitude;    // Get current GPS reading
        
        current_time_ = node_clock_->now();  // Get time elapsed since node initialization

        if (previous_gnss_[1] != 0.0) {  // Atleast two GPS readings

            // Get the distance between current and previous GPS reading

            // Convert coordinates from degrees to radians
            float current_lat = current_gnss_[0] * M_PIl / 180.0;
            float current_lon = current_gnss_[1] * M_PIl / 180.0;

            float previous_lat = previous_gnss_[0] * M_PIl / 180.0;
            float previous_lon = previous_gnss_[1] * M_PIl / 180.0;

            // Haversine formula: distance between two points on earth
            float u = sin((current_lat - previous_lat) / 2);
            float v = sin((current_lon - previous_lon) / 2);
            float dist = 2.0 * r * asin(sqrt(u * u + cos(previous_lat) * cos(current_lat) * v * v));

            // Calculate average speed

            float delta_time = abs((float)((current_time_ - previous_time_).nanoseconds()) /
                                  pow(10, 9));  // Time difference in seconds, with nanosecond precision
            float velocity = dist / delta_time;  // m/s

            message.velocity = velocity;  // Data to publish

            if (current_gnss_[0] != previous_gnss_[0] ||
                current_gnss_[1] != previous_gnss_[1]) {  // Print only if position changed since last
                RCLCPP_INFO(this->get_logger(), "velocity: '%f'", velocity);
                RCLCPP_INFO(this->get_logger(), "Lat: '%lf', Lon: '%lf'", current_gnss_[0], current_gnss_[1]);
                RCLCPP_INFO(this->get_logger(), "delta time: '%f'", (float)delta_time);
                RCLCPP_INFO(this->get_logger(), "distance: '%f'", (float)dist);

                rclcpp::sleep_for(std::chrono::nanoseconds(1));  // To have enough time to publish
                publisher_->publish(message);
            }
        } else  // Print start position
            RCLCPP_INFO(this->get_logger(), "Lat: '%lf', Lon: '%lf'", current_gnss_[0], current_gnss_[1]);

        // Save current GPS readings
        previous_gnss_[0] = current_gnss_[0];
        previous_gnss_[1] = current_gnss_[1];
        previous_time_ = current_time_;
    }

    // Declaration of fields
    rclcpp::Subscription<GNSS_MSG>::SharedPtr subscriber_pose_;
    rclcpp::Publisher<VELOCITY_MSG>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr node_clock_;
    float  previous_gnss_[2];
    rclcpp::Time current_time_;
    rclcpp::Time previous_time_;
    float r = 6371000;  // Earth radius in meters
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);                         // Init ROS 2
    rclcpp::spin(std::make_shared<VelocityCalculation>());  // Process data from the node, incl. timer callbacks
    rclcpp::shutdown();
    return 0;
}
