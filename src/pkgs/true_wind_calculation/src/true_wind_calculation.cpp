#include <chrono>
#include <iostream>
#include <memory>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <autosail_message/msg/wind_message.hpp>
#include <autosail_message/msg/pose_message.hpp>

#define POSE_TOPIC "/position/pose"
#define APPARENT_WIND_TOPIC "/sensor/apparent_wind"
#define TRUE_WIND_TOPIC "/sensor/true_wind"

#define POSE_MSG autosail_message::msg::PoseMessage
#define APPARENT_WIND_MSG autosail_message::msg::WindMessage
#define TRUE_WIND_MSG autosail_message::msg::WindMessage

using namespace std::chrono_literals;
using namespace rclcpp;
using std::placeholders::_1;

float boat_velocity = 0.0;
int apparent_wind_angle = 0;
float apparent_wind_speed = 0.0;

class TrueWindCalculation : public rclcpp::Node {
   public:
    TrueWindCalculation() : Node("true_wind_node") {
        subscriber_pose_ = this->create_subscription<POSE_MSG>(
            POSE_TOPIC, 50, std::bind(&TrueWindCalculation::pose_callback, this, _1)); 

        subscriber_apparent_wind_ = this->create_subscription<APPARENT_WIND_MSG>(
            APPARENT_WIND_TOPIC, 50, std::bind(&TrueWindCalculation::apparent_wind_callback, this, _1)); 

        publisher_true_wind_ = this->create_publisher<TRUE_WIND_MSG>(TRUE_WIND_TOPIC, 50);

        timer_ = this->create_wall_timer(100ms, std::bind(&TrueWindCalculation::true_wind_callback, this));
    }

   private:
    void pose_callback(const POSE_MSG::SharedPtr msg) {
        boat_velocity = msg->velocity;
    }

    void apparent_wind_callback(const APPARENT_WIND_MSG::SharedPtr msg) {
        apparent_wind_angle = msg->wind_angle;
        apparent_wind_speed = msg->wind_speed;
    }

    void true_wind_callback() {
        auto message = TRUE_WIND_MSG();
        message.wind_angle = sqrt(
            pow(cos(apparent_wind_angle)*apparent_wind_speed+boat_velocity, 2)+
            pow(sin(apparent_wind_angle)*apparent_wind_speed, 2));

        message.wind_speed = atan2(
            sin(apparent_wind_angle)*apparent_wind_speed,
            cos(apparent_wind_angle)*apparent_wind_speed+boat_velocity);

        publisher_true_wind_->publish(message);
    }
    // Defines
    rclcpp::Subscription<POSE_MSG>::SharedPtr subscriber_pose_;
    rclcpp::Subscription<APPARENT_WIND_MSG>::SharedPtr subscriber_apparent_wind_;
    rclcpp::Publisher<TRUE_WIND_MSG>::SharedPtr publisher_true_wind_;
    TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrueWindCalculation>());
    rclcpp::shutdown();
    return 0;
}