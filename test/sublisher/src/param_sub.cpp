#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
    MinimalSubscriber()
    : Node("param_sub")
    {
        this->declare_parameter<std::string>("my_topic", "topic"); //(paramName, default)
        this->get_parameter("my_topic", param_topic_); //(paramName, type)
        subscription_ = this->create_subscription<std_msgs::msg::Float32>( //Constructor uses the node's create_subscription class for callbacks
        param_topic_, 50, std::bind(&MinimalSubscriber::topic_callback, this, _1)); //No timer, instant response
    }

    private: //Name and type must match with publisher
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const //Receives the string message data published over the topic
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data); //Write to console using macro
    }
    //Field declaration
    std::string param_topic_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); //Prep to receive from pub
  rclcpp::shutdown();
  return 0;
}
