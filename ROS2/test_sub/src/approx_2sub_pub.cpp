#include <cstring>
#include <memory>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define SUB_TOPIC1 "/position/IMU"
#define SUB_TOPIC2 "/position/GPS"
#define PUB_TOPIC "/position/Test"

#define STD_MSG std_msgs::msg::Float32MultiArray

using std::placeholders::_1;
using std::placeholders::_2;

// For all who enter here, good luck

class ExactTimeSubscriber : public rclcpp::Node {
   public:
    ExactTimeSubscriber() : Node("exact_time_subscriber") {
        subscription_temp_1_.subscribe(this, SUB_TOPIC1);
        subscription_temp_2_.subscribe(this, SUB_TOPIC2);

        typedef message_filters::sync_policies::ApproximateTime<STD_MSG, STD_MSG> approximate_policy;
        message_filters::Synchronizer<approximate_policy> syncApproximate(approximate_policy(1), subscription_temp_1_,
                                                                          subscription_temp_2_);
        // syncApproximate.setMaxIntervalDuration(rclcpp::Duration(100, 0));  // Added after a comment
        syncApproximate.registerCallback(std::bind(&ExactTimeSubscriber::topic_callback, this, _1, _2));
    }

   private:
    void topic_callback(const STD_MSG::ConstSharedPtr &tmp_1, const STD_MSG::ConstSharedPtr &tmp_2) const {
        printf("I read: [%f] and [%f]\n", tmp_1->data[0], tmp_2->data[0]);
    }
    message_filters::Subscriber<STD_MSG> subscription_temp_1_;
    message_filters::Subscriber<STD_MSG> subscription_temp_2_;
    // std::shared_ptr<message_filters::sync_policies::ApproximateTime<STD_MSG, STD_MSG>> sync_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExactTimeSubscriber>());
    rclcpp::shutdown();

    return 0;
}