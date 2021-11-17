// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalPubSub : public rclcpp::Node  // Create node class by inheriting
{                                          // this refers to the MinimalPublisher node
   public:
    MinimalPubSub()  // The public constructor intits node
        : Node("pubsub") {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(  // Constructor uses the node's
                                                                         // create_subscription class for callbacks
            "/test", 50, std::bind(&MinimalPubSub::topic_callback, this, _1));  // No timer, instant response

        publisher_ = this->create_publisher<std_msgs::msg::String>("/topic", 50);

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
    }

   private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto message = std_msgs::msg::String();
        message.data = msg->data;

        currTime_ = nodeTime_->now();
        rclcpp::sleep_for(std::chrono::nanoseconds(1));
        publisher_->publish(message);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time currTime_;
    rclcpp::Time prevTime_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPubSub>());
    rclcpp::shutdown();
    return 0;
}
