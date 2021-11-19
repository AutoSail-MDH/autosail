#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("subber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>( //Constructor uses the node's create_subscription class for callbacks
      "topic", 50, std::bind(&MinimalSubscriber::topic_callback, this, _1)); //No timer, instant response
  }

private: //Name and type must match with publisher
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const //Receives the string message data published over the topic
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str()); //Write to console using macro
  }
  //Field declaration
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); //Prep to receive from pub
  rclcpp::shutdown();
  return 0;
}
