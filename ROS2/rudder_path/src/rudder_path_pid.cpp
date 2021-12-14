
// example gotten from avalible examples within ROS, modified by Erik Lindgren

#include <math.h>

#include <chrono>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Topics for which to read from and publish to
#define SUB_TOPIC_1 "/position/fusion"
#define SUB_TOPIC_2 "/position/GPS"
#define SUB_TOPIC_3 "/position/GOAL"
#define PUB_TOPIC "/rudder/ANGLE"

// Defines to make the code cleaner and more readable
#define LEFT_TURN 1
#define RIGHT_TURN 2

// The angle limit as specified by hardware
#define ANGLE_LIM 45

// A define to easier change which message type is used, since the expression appears everywhere
#define STD_MULTIFLOAT std_msgs::msg::Float32MultiArray
#define STD_FLOAT std_msgs::msg::Float32

// Used to regulate the PID
#define THRESHHOLD 5.0

// Namespaces to make the code far more readable
using namespace std::chrono_literals;
using namespace rclcpp;
using std::placeholders::_1;

// Save the latest value of the different topics

// Current position in Lat/long
float c_lat = 0.0;
float c_long = 0.0;

// Goal position is Lat/Long
float g_lat = 0.0;
float g_long = 0.0;

// Current heading in rad
float yaw = 0.0;

float pid = THRESHHOLD;

// Final rudder angle
float rud_ang = 0.0;
// Counter
int c = 0;

// Function declerations

float GetBearing(void);
float SetRudderAng(float Angle, int dir);
int AngleDir(float heading, float bearing);
float AngleToGoal(float heading, float bearing);

class MinimalSubPub : public rclcpp::Node {
   public:
    MinimalSubPub() : Node("rudder_angle") {
        // Create a parameter so the threshhold for the PID can change during runtime
        this->declare_parameter<float>("p", THRESHHOLD);
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
    void IMU_callback(const STD_MULTIFLOAT::SharedPtr msg) { yaw = msg->data[2]; }

    // Get current position in Lat/Long
    void GPS_callback(const STD_MULTIFLOAT::SharedPtr msg) {
        c_lat = msg->data[0] * (M_PI / 180.0);
        c_long = msg->data[1] * (M_PI / 180.0);
    }
    // Get current goal position in Lat/Long
    void GOAL_callback(const STD_MULTIFLOAT::SharedPtr msg) {
        g_lat = msg->data[0] * M_PIl / 180.0;
        g_long = msg->data[1] * M_PIl / 180.0;
    }

    // Larger calback to compute distance, unify the heading and bearing, as well as set which angle to set the rudder
    // to
    void topic_callback() {
        auto message = STD_FLOAT();
        this->get_parameter("p", pid);

        // Yaw value is between -180 to 180, convert to between 0 and 360
        float heading = yaw;
        if (heading < 0) {
            heading = 180 + (180 + heading);
        }

        float bearing = GetBearing();

        message.data = SetRudderAng(AngleToGoal(heading, bearing), AngleDir(heading, bearing));

        // Uncomment the row below to see the live print of the current heading and bearing
        printf("[%d] [Heading: %.2f] [Bearing %.2f] [Rudder Angle %.2f]\n", c++, heading, bearing, rud_ang);

        // Publish the rudder angle to a topic
        message.data = rud_ang;
        currTime_ = nodeTime_->now();
        sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(message);
    }
    Subscription<STD_MULTIFLOAT>::SharedPtr subscriber_IMU;
    Subscription<STD_MULTIFLOAT>::SharedPtr subscriber_GPS;
    Subscription<STD_MULTIFLOAT>::SharedPtr subscriber_GOAL;
    Publisher<STD_FLOAT>::SharedPtr publisher_;
    Clock::SharedPtr nodeTime_;
    Time currTime_;
    Time prevTime_;
    TimerBase::SharedPtr timer_;
};

/**
 * @brief Compute initial bearing between true north and the distance vector between goal pos and current pos and
 * convert it to degrees
 *
 * @return the bearing as a float
 */

float GetBearing(void) {
    //
    float bearing = atan2(sin(g_long - c_long) * cos(g_lat),
                          cos(c_lat) * sin(g_lat) - sin(c_lat) * cos(g_lat) * cos(g_long - c_long));

    // Convert rad to degrees, and add 360 degrees to normalize it. Then perform mod 360 to not get a
    // degree above 360.
    bearing = ((bearing * 180) / M_PIl) + 360;
    bearing = fmod(bearing, 360);

    return bearing;
}

/**
 * @brief Takes the angle between the current boat heading and the goal, and sets the angle for the rudder
 *
 * @param Angle Angle between the heading and bearing
 * @param dir direction to turn
 * @return returns the angle to set the rudder to
 */

float SetRudderAng(float Angle, int dir) {
    if (Angle < pid) {
        rud_ang = 0;
    } else {
        rud_ang = Angle / 2;
    }

    if (rud_ang > ANGLE_LIM) {
        rud_ang = ANGLE_LIM;
    }

    // Change the sign of the angle so the rudder angle is set in the right direction
    if (dir == LEFT_TURN) {
        rud_ang = (-1) * rud_ang;
    }
    return rud_ang;
}

/**
 * @brief Computes if turning clockwise or counter clickwise is better to faster reach the goal position
 *
 * @param heading the angle which the boat is currently facing
 * @param bearing the angle which the goal position is
 * @return returns 1 if the turn should be left(CCW), and 2 is the turn should be right(CW)
 */

int AngleDir(float heading, float bearing) {
    float prov = bearing - heading;
    float turn = 0.0;
    // Math to compute which way to turn.
    // From: https://math.stackexchange.com/questions/1366869/calculating-rotation-direction-between-two-angles
    if ((prov <= 180) && prov > -180) {
        turn = prov;
    } else if (prov > 180) {
        turn = prov - 360;
    } else if (prov <= -180) {
        turn = prov + 360;
    }

    int dir = 0;

    // Left turn
    if (turn < 0) {
        dir = LEFT_TURN;
        // Right turn
    } else {
        dir = RIGHT_TURN;
    }

    return dir;
}

/**
 * @brief Computes the angle between the heading and bearing
 *
 * @param heading the angle which the boat is currently facing
 * @param bearing the angle which the goal currently is
 * @return the angle as a float
 */

float AngleToGoal(float heading, float bearing) {
    float ret = abs(bearing - heading);
    ret = fmod(ret, 360);

    if (ret > 180) {
        ret = 360 - ret;
    }

    return ret;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubPub>());
    rclcpp::shutdown();
    return 0;
}
