// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Modified by Erik Lindgren
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
int fault = 0;
int c = 0;

class MinimalSubscriber : public rclcpp::Node {
   public:
    MinimalSubscriber() : Node("gps_test") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/position/GPS", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

   private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const {
        // Checking if the lat/long values are not correct, and prints a fault if they are incorrect
        if ((int)floor(msg->data.data()[0]) != 59 || (int)floor(msg->data.data()[1]) != 16) {
            fault++;
            RCLCPP_INFO(this->get_logger(), "[%d] [Lat: %f - Long: %f] Faults: [%d]", c, msg->data.data()[0],
                        msg->data.data()[1], fault);
        }
        c++;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
