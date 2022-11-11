#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
//#include <autosail_message/msg/sail_angle_message.h>
//#include <autosail_message/msg/sail_furl_message.h>
#include <autosail_message/msg/rudder_control_message.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "driver/mcpwm.h"

#ifdef ESP_PLATFORM
#include "driver/i2c.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK)) {                                                   \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            esp_restart();                                                               \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK)) {                                                     \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000
// microseconds
#define SERVO_MIN_PULSEWIDTH_US 800     // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2200    // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90             // Maximum angle in degree upto which servo can rotate
#define SERVO_MIN_DEGREE 0            // Minimum angle
#define SERVO_PULSE_GPIO_SAIL 19        // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_RUDDER 18      // GPIO connects to the PWM signal line

#define MS_DELAY (10)  // Callback delay

rcl_subscription_t sub_sail;
rcl_subscription_t sub_rudder;
rcl_subscription_t sub_furl;
//autosail_message__msg__SailAngleMessage angle_msg;
autosail_message__msg__RudderControlMessage rudder_msg;
//autosail_message__msg__SailFurlMessage furl_msg;

struct timeval startTime;
struct timeval currTime;
struct timeval prevTimeSail;
struct timeval prevTimeRudder;
struct timeval prevTimeFurl;

/*
void sail_callback(const void *msgin) {
    const autosail_message__msg__SailAngleMessage *msg = (const autosail_message__msg__SailAngleMessage *)msgin;

    // Use angle to calculate PMW

    int32_t angle = msg->sail_angle;  // Data sent in degrees
    printf("Sail angle received: %d\r\n", angle);

    uint32_t duty_us =
        (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) +
        SERVO_MIN_PULSEWIDTH_US;

    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_us));
    vTaskDelay(pdMS_TO_TICKS(MS_DELAY));  // Add delay, since it takes time for servo to rotate, generally
                                          // 100ms/60degree rotation under 5V power supply

    printf("Sail duty cycle set to: %f\r\n", (float)duty_us);

    gettimeofday(&prevTimeSail, NULL);
}
*/

void rudder_callback(const void *msgin) {
    const autosail_message__msg__RudderControlMessage *msg = (const autosail_message__msg__RudderControlMessage *)msgin;

    // Use angle to calculate PMW

    int32_t angle = msg->rudder_angle;  // Data sent in degrees
    printf("Rudder angle received: %d\r\n", angle);
    /*
    if (angle > 45) {
        angle = 45;
    }
    else if (angle < -45){
        angle = 45;
    }
    */

    
    uint32_t duty_us = ((angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)) + SERVO_MIN_PULSEWIDTH_US;
    
    /*
    uint32_t duty_us =
        (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) +
        SERVO_MIN_PULSEWIDTH_US;
    */

    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty_us));
    vTaskDelay(pdMS_TO_TICKS(MS_DELAY));  // Add delay, since it takes time for servo to rotate, generally
                                          // 100ms/60degree rotation under 5V power supply

    printf("Rudder duty cycle set to: %f\r\n", (float)duty_us);
    

    gettimeofday(&prevTimeRudder, NULL);
}

/*
void furl_callback(const void *msgin) {
    gettimeofday(&prevTimeFurl, NULL);
}
*/

void appMain(void* arg) {

    while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1));

    gettimeofday(&startTime, NULL);
    prevTimeSail = startTime;
    prevTimeRudder = startTime;


    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

    /*
    RCCHECK(rclc_subscription_init_default(&sub_sail, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, SailAngleMessage),
                                           "/actuator/sail"));
    */
    vTaskDelay(500 / portTICK_PERIOD_MS);
    RCCHECK(rclc_subscription_init_default(&sub_rudder, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, RudderControlMessage),
                                           "/actuator/rudder"));
    
    /*
    vTaskDelay(500 / portTICK_PERIOD_MS);
    RCCHECK(rclc_subscription_init_default(&sub_furl, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, SailFurlMessage),
                                           "/actuator/furl"));
    */

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    //RCCHECK(rclc_executor_add_subscription(&executor, &sub_sail, &angle_msg, &sail_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_rudder, &rudder_msg, &rudder_callback, ON_NEW_DATA));
    //RCCHECK(rclc_executor_add_subscription(&executor, &sub_furl, &furl_msg, &furl_callback, ON_NEW_DATA));

    // configure motor control pulse width modulator (MCPWM)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A,
                    SERVO_PULSE_GPIO_SAIL);  // To drive a RC servo, one MCPWM generator is enough
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A,
                    SERVO_PULSE_GPIO_RUDDER);  // To drive a RC servo, one MCPWM generator is enough

    mcpwm_config_t pwm_config = {
        .frequency = 50,  // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,      // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        gettimeofday(&currTime, NULL);
        // if (((prevTimeSail.tv_sec > 1) && (prevTimeRudder.tv_sec > 1)) || RMW_RET_OK != rmw_uros_ping_agent(1000, 1))
            // if (((currTime.tv_sec - prevTimeSail.tv_sec) >= 3) || ((currTime.tv_sec - prevTimeRudder.tv_sec) >= 3)) {
                // settimeofday(&startTime, NULL);
                // esp_restart();
            // }
        // usleep(100);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&sub_sail, &node));
    RCCHECK(rcl_subscription_fini(&sub_rudder, &node));
    RCCHECK(rcl_subscription_fini(&sub_furl, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support));
}
