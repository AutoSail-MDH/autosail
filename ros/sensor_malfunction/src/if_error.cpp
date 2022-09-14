#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define FATAL -1000

class MinimalSubscriber : public rclcpp::Node {
   public:
    MinimalSubscriber() : Node("param_sub") {
        //Let the system start before checking for erros
        rclcpp::Rate r(std::chrono::seconds(7));
        r.sleep();

        this->declare_parameter<std::string>("my_topic", "topic");  //(paramName, default)
        this->get_parameter("my_topic", param_topic_);              //(paramName, type)

        this->declare_parameter<std::int32_t>("my_DL", 20);
        this->get_parameter("my_DL", param_DL_);

        this->declare_parameter<std::int32_t>("my_iteration", 20);
        this->get_parameter("my_iteration", param_iteration_);

        subscription_ = this->create_subscription<
            std_msgs::msg::Float32MultiArray>(  // Constructor uses the node's create_subscription class for callbacks
            param_topic_, 50, std::bind(&MinimalSubscriber::topic_callback, this, _1));  // No timer, instant response

        timer_ = this->create_wall_timer(  // Init timer and start timer_callback to execute 2 times/s
            500ms, std::bind(&MinimalSubscriber::timer_callback, this));

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
        prevTime_ = nodeTime_->now();
    }

   private:  // Name and type must match with publisher
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        auto message = msg->data;
        auto pubName = subscription_->get_topic_name();

        prevTime_ = nodeTime_->now();  // Get time elapsed since node initialization

        if (message[0] == FATAL) RCLCPP_FATAL(this->get_logger(), "INITIATE SHUTDOWN, %s OUT OF FUNCTION", pubName);
    }

    void timer_callback()  // Function for setting messages and publishing
    {
        auto pubName = subscription_->get_topic_name();  // param_topic_.c_str();
        auto currTime = nodeTime_->now();

        if (connected == 1) {
            if ((currTime - prevTime_).seconds() > 1) {  // If no values for certain amount of time
                connected = 0;
                iteration++;
            } else {
            }  // RCLCPP_INFO(this->get_logger(), "Topic %s/%s functioning", pubSpace, pubName);
        } else {
            if ((currTime - prevTime_).seconds() < 1) connected = 1;
            if (iteration >= param_iteration_)
                RCLCPP_FATAL(this->get_logger(), "INITIATE SHUTDOWN, %s OUT OF FUNCTION", pubName);
            else if ((currTime - prevTime_).seconds() < param_DL_)
                RCLCPP_WARN(this->get_logger(), "WARNING, %s MALFUNCTION", pubName);
            else
                RCLCPP_FATAL(this->get_logger(), "INITIATE SHUTDOWN, %s OUT OF FUNCTION", pubName);
        }
    }
    // Field declaration
    std::string param_topic_;
    std::int32_t param_DL_;
    std::int32_t param_iteration_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time prevTime_;
    int32_t connected = 1;
    int32_t iteration = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());  // Prep to receive from pub
    rclcpp::shutdown();
    return 0;
}
