#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <autosail_message/msg/measured_sail_angle_message.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "components/INA219/include/INA219.h"
#include "components/INA219/INA219.cpp"

#ifdef ESP_PLATFORM
#include "driver/i2c.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define FATAL -1000
#define rec 2      // How often data is sent to topic

//Error check function. If error found restart microcontroller
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

rcl_publisher_t publisher_boom_angle;
autosail_message__msg__MeasuredSailAngleMessage msg_boomangle;

INA219 ina;

extern "C" {
void appMain(void* arg);
void InitBoomReading();
}

void InitBoomReading() {
    ina.begin(I2C_NUM_1,GPIO_NUM_21,GPIO_NUM_22);

    vTaskDelay(500 / portTICK_PERIOD_MS);
}

float getTrueSailAngle(){
    //measure the voltage from the angle sensor and convert to current(mA)
    float angle_current_mA = ina.shuntVoltage()*10000;//convert to current
    angle_current_mA = abs(angle_current_mA);

    //boom angle can simply be described with a linear formula. 4mA = 0/360deg. 8mA = 90deg 
    float measured_boom_angle = 22.5*angle_current_mA-90;
    
    //offset depending on how the sensor is positioned
    measured_boom_angle -= 90;

    //rewrite angle to be in the span of +-180 degrees
    if(measured_boom_angle > 180)
        measured_boom_angle-=360;

    return measured_boom_angle;
}

void BoomCallback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        
        float measured_boom_angle = getTrueSailAngle();

        msg_boomangle.measured_sail_angle = measured_boom_angle;

        // micro-ROS to publish to topic
        RCCHECK(rcl_publish(&publisher_boom_angle, &msg_boomangle, NULL));
    }
}
void appMain(void* arg) {

    InitBoomReading();

    // Setup for micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
       
    // create boom angle sensor node
    rcl_node_t node_boom_angle;
    RCCHECK(rclc_node_init_default(&node_boom_angle, "boom_angle_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_boom_angle, &node_boom_angle, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, MeasuredSailAngleMessage), "/sensor/boom_angle"));
    // create boom angle sensor timer
    rcl_timer_t timer_boom_angle;
    RCCHECK(rclc_timer_init_default(&timer_boom_angle, &support, RCL_MS_TO_NS(100), BoomCallback));
    // create boom angle sensor executor
    rclc_executor_t executor_boom_angle = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_boom_angle, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_boom_angle, &timer_boom_angle));
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    while (1)
    {
        rclc_executor_spin_some(&executor_boom_angle, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_boom_angle, &node_boom_angle));
    
    // destroy node
    RCCHECK(rcl_node_fini(&node_boom_angle));
}
