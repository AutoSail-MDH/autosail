
// example gotne from avalible examples within ROS, modified by Erik Lindgren

#include <math.h>

#include <chrono>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Topics for which to read from and publish to
#define SUB_TOPIC_1 "/position/IMU"
#define SUB_TOPIC_2 "/position/GPS"
#define SUB_TOPIC_3 "/position/GOAL"
#define PUB_TOPIC "/rudder/ANGLE"

// A define to easier change which message type is used, since the expression appears everywhere
#define STD_MULTIFLOAT std_msgs::msg::Float32MultiArray
#define STD_FLOAT std_msgs::msg::Float32

// Used to regulate the PID
#define THRESHHOLD 5

using namespace std::chrono_literals;
using std::placeholders::_1;

// Save the latest value of the different topics

// Current position in Lat/long
float c_lat = 0.0;
float c_long = 0.0;

// Goal position is Lat/Long
float g_lat = 60.0000 * (M_PI / 180.0);

float g_long = 16.0000 * (M_PI / 180.0);

// Current heading in rad
float yaw = 0.0;

float rud_ang = 0;
// float Angle2Goal = 0;
// Counter
int c = 0;
float earth_rad = 6371000.0;
int rightTurn = 0;

class MinimalSubPub : public rclcpp::Node {
   public:
    MinimalSubPub() : Node("subpub") {
        // Create three subscribers for 3 different topics. each is bound to a custom callback
        subscriber_IMU = this->create_subscription<STD_MULTIFLOAT>(

            SUB_TOPIC_1, 50, std::bind(&MinimalSubPub::IMU_callback, this, _1));
        subscriber_GPS = this->create_subscription<STD_MULTIFLOAT>(

            SUB_TOPIC_2, 50, std::bind(&MinimalSubPub::GPS_callback, this, _1));
        subscriber_GOAL = this->create_subscription<STD_MULTIFLOAT>(

            SUB_TOPIC_3, 50, std::bind(&MinimalSubPub::GOAL_callback, this, _1));

        // Create publisher
        publisher_ = this->create_publisher<STD_FLOAT>(PUB_TOPIC, 50);

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
        timer_ = this->create_wall_timer(100ms, std::bind(&MinimalSubPub::topic_callback, this));
    }

   private:
    // Get current heading
    void IMU_callback(const STD_MULTIFLOAT::SharedPtr msg) { yaw = msg->data[0]; }

    // Get current position in Lat/Long
    void GPS_callback(const STD_MULTIFLOAT::SharedPtr msg) {
        c_lat = 59.25 * (M_PI / 180.0);
        c_long = 16.33 * (M_PI / 180.0);
    }
    // Get current goal position in Lat/Long
    void GOAL_callback(const STD_MULTIFLOAT::SharedPtr msg) {
        // g_lat = msg->data[0] * M_PIl / 180.0;
        // g_long = msg->data[1] * M_PIl / 180.0;
        g_lat = 60.0000 * (M_PI / 180.0);
        g_long = 16.0000 * (M_PI / 180.0);
    }

    // Larger calback to compute distance, unify the heading and bearing, as well as set which angle to set the rudder
    // to
    void topic_callback() {
        auto message = STD_FLOAT();

        float heading = 0;

        // Calculate distance
        float u = sin((g_lat - c_lat) / 2.0);
        float v = sin((g_long - c_long) / 2.0);
        float dDist = 2.0 * earth_rad * asin(sqrt(u * u + cos(c_lat) * cos(g_lat) * v * v));

        // Yaw value is between -180 to 180, convert to between 0 and 360
        heading = yaw;
        if (heading < 0) {
            heading = 180 + (180 + heading);
        }

        // Compute initial bearing between true north and the distance vector between goal pos and current pos
        float bearing = atan2(sin(g_long - c_long) * cos(g_lat),
                              cos(c_lat) * sin(g_lat) - sin(c_lat) * cos(g_lat) * cos(g_long - c_long));

        // Convert rad to degrees, and add 360 degrees to normalize it. Then perform mod 360 to not get a
        // degree above 360.
        bearing = ((bearing * 180) / M_PIl) + 360;
        bearing = fmod(bearing, 360);

        // Compute angle to goal and mod it with 360.

        float Angle2Goal = abs(bearing - heading);
        Angle2Goal = fmod(Angle2Goal, 360);
        if (Angle2Goal > 180) {
            Angle2Goal = 360 - Angle2Goal;
        }

        // "PID - ID"
        if (Angle2Goal < THRESHHOLD) {
            rud_ang = 0;
        } else {
            rud_ang = Angle2Goal / 2;
        }

        if (rud_ang > 60) {
            rud_ang = 60;
        }

        if (rightTurn == 1) {
            rud_ang = (-1) * rud_ang;
        }

        printf(
            "[%d] [Distance to goal: %.2f m] [Boat Heading: %.2f deg] [Final bearing: %.2f deg]  [Angle to goal is: "
            "%.2f deg] [Rudder Angle: %.2f deg]\n",
            c++, dDist, heading, bearing, Angle2Goal, rud_ang);
        // Publish the rudder angle to a topic
        message.data = rud_ang;
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
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubPub>());
    rclcpp::shutdown();
    return 0;
}
