#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
    MinimalSubscriber()
    : Node("param_sub")
    {
        this->declare_parameter<std::string>("my_topic", "topic"); //(paramName, default)
        this->get_parameter("my_topic", param_topic_); //(paramName, type)

        this->declare_parameter<std::int32_t>("my_DL", 20);
        this->get_parameter("my_DL", param_DL_);

        timer_ = this->create_wall_timer( //Init timer and start timer_callback to execute 2 times/s
            500ms, std::bind(&MinimalSubscriber::timer_callback, this));
        
        nodeTime_ = this->get_clock(); //Create clock starting at the time of node creation
    }

    void timer_callback() //Function for setting messages and publishing
    {
        auto pubName = param_topic_.c_str();
        auto pubCount = this->count_publishers(pubName);
        auto pubSpace = this->get_namespace();
        //RCLCPP_INFO(this->get_logger(), "Count: %d, Name: %s", pubCount, pubName);
        //RCLCPP_INFO(this->get_logger(), "Deadline: %d", param_DL_);

        if (connected == 1) {
            if (pubCount < 1) {
                prevTime_ = nodeTime_->now();
                connected = 0;
            }
            else
                {}//RCLCPP_INFO(this->get_logger(), "Topic %s/%s functioning", pubSpace, pubName);
        }
        else {
            if (pubCount > 0)
                connected = 1;
            if ((nodeTime_->now() - prevTime_).seconds() < param_DL_)
                RCLCPP_WARN(this->get_logger(), "WARNING, %s/%s MALFUNCTION", pubSpace, pubName);
            else
                RCLCPP_FATAL(this->get_logger(), "INITIATE SHUTDOWN, %s/%s OUT OF FUNCTION", pubSpace, pubName);
        }
    }
    //Field declaration
    std::string param_topic_;
    std::int32_t param_DL_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time prevTime_;
    int32_t connected = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); //Prep to receive from pub
  rclcpp::shutdown();
  return 0;
}
