#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "autosail_message/msg/gnss_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define FATAL -1000

class GNSSError : public rclcpp::Node {
   public:
    GNSSError() : Node("param_sub") {
        //Let the system start before checking for erros
        rclcpp::Rate r(std::chrono::seconds(7));
        r.sleep();

        this->declare_parameter<std::string>("topic", "topic");  //(paramName, default)
        this->get_parameter("topic", param_topic_);              //(paramName, type)

        this->declare_parameter<std::int32_t>("timeout", 20);
        this->get_parameter("timeout", param_timeout_);

        this->declare_parameter<std::int32_t>("iteration", 20);
        this->get_parameter("iteration", param_iteration_);

        subscription_ = this->create_subscription<
            autosail_message::msg::GNSSMessage>(  // Constructor uses the node's create_subscription class for callbacks
            param_topic_, 50, std::bind(&GNSSError::topic_callback, this, _1));  // No timer, instant response

        timer_ = this->create_wall_timer(  // Init timer and start timer_callback to execute 2 times/s
            500ms, std::bind(&GNSSError::timer_callback, this));

        node_time_ = this->get_clock();  // Create clock starting at the time of node creation
        previous_time_ = node_time_->now();
    }

   private:  // Name and type must match with publisher
    void topic_callback(const autosail_message::msg::GNSSMessage::SharedPtr msg) {
        auto message = (int)msg->gps_fix;
        auto gnss_topic_name = subscription_->get_topic_name();

        previous_time_ = node_time_->now();  // Get time elapsed since node initialization

        if (message == FATAL) RCLCPP_FATAL(this->get_logger(), "INITIATE SHUTDOWN, %s OUT OF FUNCTION", gnss_topic_name);
    }

    void timer_callback()  // Function for setting messages and publishing
    {
        auto gnss_topic_name = subscription_->get_topic_name();  // param_topic_.c_str();
        auto current_time = node_time_->now();

        if (connected == 1) {
            if ((current_time - previous_time_).seconds() > 1) {  // If no values for certain amount of time
                connected = 0;
                iteration++;
            } else {
                //RCLCPP_INFO(this->get_logger(), "Topic %s/%s functioning", pubSpace, pubName);
            }        
        } else {
            if ((current_time - previous_time_).seconds() < 1) connected = 1;
            if (iteration >= param_iteration_)
                RCLCPP_FATAL(this->get_logger(), "INITIATE SHUTDOWN, %s OUT OF FUNCTION", gnss_topic_name);
            else if ((current_time - previous_time_).seconds() < param_timeout_)
                RCLCPP_WARN(this->get_logger(), "WARNING, %s MALFUNCTION", gnss_topic_name);
            else
                RCLCPP_FATAL(this->get_logger(), "INITIATE SHUTDOWN, %s OUT OF FUNCTION", gnss_topic_name);
        }
    }
    // Field declaration
    std::string param_topic_;
    std::int32_t param_timeout_;
    std::int32_t param_iteration_;

    rclcpp::Subscription<autosail_message::msg::GNSSMessage>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr node_time_;
    rclcpp::Time previous_time_;
    int32_t connected = 1;
    int32_t iteration = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSError>());  // Prep to receive from pub
    rclcpp::shutdown();
    return 0;
}
