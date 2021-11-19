
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

float Angle2Goal = 0;
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

        float yaw = msg->data[0];
        float heading = 0;

        float curr_lat = 59.370687 * M_PIl / 180.0;
        float curr_lon = 16.336636 * M_PIl / 180.0;
        float goal_lat = 60.000000 * M_PIl / 180.0;
        float goal_lon = 16.000000 * M_PIl / 180.0;

        float u = sin((goal_lat - curr_lat) / 2);
        float v = sin((goal_lon - curr_lon) / 2);
        float dDist = 2.0 * earth_rad * asin(sqrt(u * u + cos(curr_lat) * cos(goal_lat) * v * v));

        if (yaw < 0) {
            yaw = 180 + (180 + yaw);
        }

        heading = yaw;

        // Compute initial bearing between true north and the distance vector between goal pos and current pos
        float bearing = atan2(sin(goal_lon - curr_lon) * cos(goal_lat),
                              cos(curr_lat) * sin(goal_lat) - sin(curr_lat) * cos(goal_lat) * cos(goal_lon - curr_lon));

        // Convert rad to degrees, and add 360 degrees to normalize it. Then perform mod 360 to not get a
        // degree above 360.
        bearing = ((bearing * 180) / M_PIl) + 360;
        bearing = fmod(bearing, 360);

        // Compute angle to goal and mod it with 360.
        Angle2Goal = bearing + heading;
        Angle2Goal = fmod(Angle2Goal, 360);

        printf("[%d] [Boat Angle: %.2f] [Distance to goal: %.2f] [Angle to goal is: %.2f] [Final bearing: %.2f]\n", c++,
               heading, dDist, Angle2Goal, bearing);

        // Assign data to the message and publish it
        message.data = Angle2Goal;
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
