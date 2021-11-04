//Standard C++ headers
#include <chrono>
#include <memory>
#include <cstring>
#include <math.h>
//Dependencies
#include "rclcpp/rclcpp.hpp" //Common for ROS 2
#include <std_msgs/msg/float32_multi_array.hpp> //Built-in messages types for publishing data

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPubSub : public rclcpp::Node //Create node class by inheriting
{ //this refers to the MinimalPublisher node
public:
  MinimalPubSub() //The public constructor intits node
  : Node("pubsub")
  { //           node  create publisher of type String      Topic name and value
    GPS_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>( //Constructor uses the node's create_subscription class for callbacks
      "/position/IMU", 50, std::bind(&MinimalPubSub::GPS_topic_callback, this, _1)); //No timer, instant response

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/position/pospub", 50); //Init msg type, topic name and msg size

    nodeTime_ = this->get_clock();
  }

private:
  void GPS_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) //Receives the string message data published over the topic
  {
    auto message = msg->data[0]; //Test

    auto curr_GPS_ = msg->data; //Get current GPS reading
    currTime_ = nodeTime_->now(); //Get time elapsed since node initialization

    //RCLCPP_INFO(this->get_logger(), "I heard: '%lf'", curr_GPS_->data[0]); //Write to console using macro

    if (prev_GPS_.capacity() != 0) { //Atleast two GPS readings

    // Get the distance between current and previous GPS reading

    float currLat = curr_GPS_[0] * M_PIl / 180.0;
    float currLon = curr_GPS_[1] * M_PIl / 180.0;

    float prevLat = prev_GPS_[0] * M_PIl / 180.0;
    float prevLon = prev_GPS_[1] * M_PIl / 180.0;

    // P
    float currRho = r * cos(currLat);
    float currZ = r * sin(currLat);
    float currX = currRho * cos(currLon);
    float currY = currRho * sin(currLon);

    // Q
    float prevRho = r * cos(prevLat);
    float prevZ = r * sin(prevLat);
    float prevX = prevRho * cos(prevLon);
    float prevY = prevRho * sin(prevLon);

    // Dot product
    float dot = (currX * prevX + currY * prevY + currZ * prevZ);
    float cos_theta = dot / (r * r);

    float theta = acos(cos_theta);

    float dist = r * theta;

    //Calculate average speed

    auto avgTime = (currTime_ - prevTime_).nanoseconds();
    float velocity = dist / avgTime; //m/s
    //velocity = (velocity * 3600.0) / 1000.0; //km/h

    message = velocity;
    RCLCPP_INFO(this->get_logger(), "velocity: '%f'", velocity);
    }

    //Set current to previous GPS reading
    prev_GPS_ = curr_GPS_;
    prevTime_ = currTime_;

    publisher_->publish(message);
  }

  //Declaration of fields
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr GPS_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  //std_msgs::msg::Float32MultiArray::SharedPtr curr_GPS_;
  //std_msgs::msg::Float32MultiArray::SharedPtr prev_GPS_;
  rclcpp::Clock::SharedPtr nodeTime_;
  std::vector<float> prev_GPS_;
  rclcpp::Time currTime_;
  rclcpp::Time prevTime_;
  float r = 6371000; //Earth radius m
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //Init ROS 2
  rclcpp::spin(std::make_shared<MinimalPubSub>()); //Process data from the node, incl. timer callbacks
  rclcpp::shutdown();
  return 0;
}