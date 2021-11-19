//Standard C++ headers
#include <chrono>
#include <memory>
//Dependencies
#include "rclcpp/rclcpp.hpp" //Common for ROS 2
#include "std_msgs/msg/string.hpp" //Built-in messages types for publishing data

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node //Create node class by inheriting
{ //this refers to the MinimalPublisher node
public:
  MinimalPublisher() //The public constructor intits node
  : Node("pubber"), count_(0)
  { //           node  create publisher of type String      Topic name and value
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 50); //Init msg type, topic name and msg size
    timer_ = this->create_wall_timer( //Init timer and start timer_callback to execute 2 times/s
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() //Function for setting messages and publishing
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from ROS2! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); //Macro for printing published messages to console
    publisher_->publish(message);
  }
  //Declaration of fields
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //Init ROS 2
  rclcpp::spin(std::make_shared<MinimalPublisher>()); //Process data from the node, incl. timer callbacks
  rclcpp::shutdown();
  return 0;
}
