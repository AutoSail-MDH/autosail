//Standard C++ headers
#include <chrono>
#include <memory>
#include <cstring>
#include <math.h>
//Dependencies
#include "rclcpp/rclcpp.hpp" //Common for ROS 2
#include <std_msgs/msg/float32_multi_array.hpp> //Built-in message types for publishing data

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalPubSub : public rclcpp::Node //Create node class by inheriting
{ //this refers to the MinimalPublisher node
public:
  MinimalPubSub() //The public constructor intits node
  : Node("pubsub")
  {
    GPS_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>( //Constructor uses the node's create_subscription class for callbacks
      "/position/GPS", 50, std::bind(&MinimalPubSub::GPS_topic_callback, this, _1)); //No timer, instant response

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/boat/velocity", 50); //Init msg type, topic name and msg size

    nodeTime_ = this->get_clock(); //Create clock starting at the time of node creation
  }

private:
  void GPS_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    auto message = std_msgs::msg::Float32MultiArray();
    
    //RCLCPP_INFO(this->get_logger(), "Lat: '%lf', Lon: '%lf'", msg->data[0], msg->data[1]); //Write to console using macro

    auto curr_GPS_ = msg->data; //Get current GPS reading
    currTime_ = nodeTime_->now(); //Get time elapsed since node initialization

    if (prev_GPS_.capacity() != 0) { //Atleast two GPS readings

    // Get the distance between current and previous GPS reading

    //Convert coordinates from degrees to radians
    float currLat = curr_GPS_[0] * M_PIl / 180.0;
    float currLon = curr_GPS_[1] * M_PIl / 180.0;

    float prevLat = prev_GPS_[0] * M_PIl / 180.0;
    float prevLon = prev_GPS_[1] * M_PIl / 180.0;

    /*
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

    if (cos_theta > 1) //acos of val > 1 = nan
      cos_theta = 1;

    float theta = acos(cos_theta);

    float dist = r * theta;
    */

    //Haversine formula: distance between two points on earth
    float u = sin((currLat - prevLat)/2);
    float v = sin((currLon - prevLon)/2);
    float dist = 2.0 * r * asin(sqrt(u * u + cos(prevLat) * cos(currLat) * v * v));

    //Calculate average speed

    float deltaTime = abs((float)((currTime_ - prevTime_).nanoseconds()) / pow(10, 9)); //Time difference in seconds, with nanosecond precision
    float velocity = dist / deltaTime; //m/s
    //velocity = (velocity * 3600.0) / 1000.0; //km/h

    message.data = {velocity, currLat, currLon, deltaTime}; //Data to publish

    if (curr_GPS_[0] != prev_GPS_[0] || curr_GPS_[1] !=  prev_GPS_[1]) { //Print only if position changed since last
      //RCLCPP_INFO(this->get_logger(), "currLat: %f, prevLon: %f, currY: %f, prevY: %f, dot: %f, cos_theta: %f, theta: %f, dist: %f, deltaTime: %f", currLat, prevLon, currY, prevY, dot, cos_theta, theta, dist, deltaTime);
      //RCLCPP_INFO(this->get_logger(), "currLat: %f, prevLon: %f, u: %f, v: %f, dist: %f, deltaTime: %f", currLat, prevLon, u, v, dist, deltaTime);
      RCLCPP_INFO(this->get_logger(), "velocity: '%f'", velocity);
      RCLCPP_INFO(this->get_logger(), "Lat: '%lf', Lon: '%lf'", curr_GPS_[0], curr_GPS_[1]);
      RCLCPP_INFO(this->get_logger(), "deltaTime: '%f'", (float) deltaTime);
      //RCLCPP_INFO(this->get_logger(), "prevLat: '%lf', prevLon: '%lf'", prev_GPS_[0], prev_GPS_[1]);

      //while (publisher_->get_subscription_count() < 1);
      rclcpp::sleep_for(std::chrono::nanoseconds(1)); //To have enough time to publish
      publisher_->publish(message);
      }
    }
    else //Print start position
      RCLCPP_INFO(this->get_logger(), "Lat: '%lf', Lon: '%lf'", curr_GPS_[0], curr_GPS_[1]);

    //Save current GPS readings
    prev_GPS_ = curr_GPS_;
    prevTime_ = currTime_;
  }

  //Declaration of fields
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr GPS_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr nodeTime_;
  std::vector<float> prev_GPS_;
  rclcpp::Time currTime_;
  rclcpp::Time prevTime_;
  float r = 6371000; //Earth radius in meters
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //Init ROS 2
  rclcpp::spin(std::make_shared<MinimalPubSub>()); //Process data from the node, incl. timer callbacks
  rclcpp::shutdown();
  return 0;
}