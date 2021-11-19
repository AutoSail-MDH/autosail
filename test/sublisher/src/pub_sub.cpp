//Standard C++ headers
#include <chrono>
#include <memory>
//Dependencies
#include "rclcpp/rclcpp.hpp" //Common for ROS 2
#include "std_msgs/msg/int32.hpp" //Built-in messages types for publishing data

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPubSub : public rclcpp::Node //Create node class by inheriting
{ //this refers to the MinimalPublisher node
public:
  MinimalPubSub() //The public constructor intits node
  : Node("pubsub"), count_(0)
  { //           node  create publisher of type String      Topic name and value
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/test1/ROStoESP", 50); //Init msg type, topic name and msg size
    timer_ = this->create_wall_timer( //Init timer and start timer_callback to execute 2 times/s
      500ms, std::bind(&MinimalPubSub::timer_callback, this));
    
    subscription_ = this->create_subscription<std_msgs::msg::Int32>( //Constructor uses the node's create_subscription class for callbacks
      "/test1/ESPtoROS", 50, std::bind(&MinimalPubSub::topic_callback, this, _1)); //No timer, instant response
  }

private:
  void timer_callback() //Function for setting messages and publishing
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data); //Macro for printing published messages to console
    publisher_->publish(message);
  }
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const //Receives the string message data published over the topic
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg->data); //Write to console using macro
    }
  //Declaration of fields
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //Init ROS 2
  rclcpp::spin(std::make_shared<MinimalPubSub>()); //Process data from the node, incl. timer callbacks
  rclcpp::shutdown();
  return 0;
}
