
// Example written by Peter Nguyen, Modified by Erik Lindgren

#include <chrono>
#include <cmath>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Change these to your topics
#define SUB_TOPIC_1 "/position/IMU"
#define SUB_TOPIC_2 "/position/GPS"
#define SUB_TOPIC_3 "/position/GOAL"
#define PUB_TOPIC "/rudder/ANGLE"
// Change this to your message type, made this define to not have to write the long expression
#define STD_MULTIFLOAT std_msgs::msg::Float32MultiArray
#define STD_FLOAT std_msgs::msg::Float32

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

float c_lat = 0.0;
float c_long = 0.0;
float g_lat = 0.0;
float g_long = 0.0;

float yaw = 0.0;

float Angle2Goal = 0;
int c = 0;

class MinimalSubPub : public rclcpp::Node {
   public:
    MinimalSubPub() : Node("subpub") {
        subscriber_IMU = this->create_subscription<STD_MULTIFLOAT>(

            SUB_TOPIC_1, 50, std::bind(&MinimalSubPub::IMU_callback, this, _1));
        subscriber_GPS = this->create_subscription<STD_MULTIFLOAT>(

            SUB_TOPIC_2, 50, std::bind(&MinimalSubPub::GPS_callback, this, _1));
        subscriber_GOAL = this->create_subscription<STD_MULTIFLOAT>(

            SUB_TOPIC_3, 50, std::bind(&MinimalSubPub::GOAL_callback, this, _1));

        publisher_ = this->create_publisher<STD_FLOAT>(PUB_TOPIC, 50);

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
        timer_ = this->create_wall_timer(100ms, std::bind(&MinimalSubPub::topic_callback, this));
    }

   private:
    void IMU_callback(const STD_MULTIFLOAT::SharedPtr msg) { yaw = msg->data[0]; }

    void GPS_callback(const STD_MULTIFLOAT::SharedPtr msg) {
        c_lat = msg->data[0] * M_PIl / 180.0;
        c_long = msg->data[1] * M_PIl / 180.0;
    }

    void GOAL_callback(const STD_MULTIFLOAT::SharedPtr msg) {
        g_lat = msg->data[0] * M_PIl / 180.0;
        g_long = msg->data[1] * M_PIl / 180.0;
    }

    void topic_callback() {
        auto message = STD_FLOAT();

        float heading = 0;

        float u = sin((g_lat - c_lat) / 2);
        float v = sin((g_long - c_long) / 2);
        float dDist = 2.0 * earth_rad * asin(sqrt(u * u + cos(c_lat) * cos(g_lat) * v * v));

        if (yaw < 0) {
            yaw = 180 + (180 + yaw);
        }

        heading = yaw;

        // Compute initial bearing between true north and the distance vector between goal pos and current pos
        float bearing = atan2(sin(g_long - c_long) * cos(g_lat),
                              cos(c_lat) * sin(g_lat) - sin(c_lat) * cos(g_lat) * cos(g_long - c_long));

        // Convert rad to degrees, and add 360 degrees to normalize it. Then perform mod 360 to not get a
        // degree above 360.
        bearing = ((bearing * 180) / M_PIl) + 360;
        bearing = fmod(bearing, 360);

        // Compute angle to goal and mod it with 360.
        Angle2Goal = bearing + heading;
        Angle2Goal = fmod(Angle2Goal, 360);

        printf(
            "[%d] [Boat Heading: %.2f deg] [Distance to goal: %.2f m] [Angle to goal is: %.2f deg] [Final bearing: "
            "%.2f deg]\n",
            c++, heading, dDist, Angle2Goal, bearing);

        // Assign data to the message and publish it

        message.data = Angle2Goal;
        currTime_ = nodeTime_->now();
        rclcpp::sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(message);
    }
    // Defines
    rclcpp::Subscription<STD_MULTIFLOAT>::SharedPtr subscriber_IMU;
    rclcpp::Subscription<STD_MULTIFLOAT>::SharedPtr subscriber_GPS;
    rclcpp::Subscription<STD_MULTIFLOAT>::SharedPtr subscriber_GOAL;
    rclcpp::Publisher<STD_FLOAT>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time currTime_;
    rclcpp::Time prevTime_;
    rclcpp::TimerBase::SharedPtr timer_;
    float earth_rad = 6371000;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubPub>());
    rclcpp::shutdown();
    return 0;
}
