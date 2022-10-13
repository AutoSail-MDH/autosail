#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "components/I2Cdev/I2Cdev.cpp"
#include "components/I2Cdev/I2Cdev.h"


#ifdef ESP_PLATFORM
#include "driver/i2c.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define PIN_SDA 21
#define PIN_CLK 22

#define FATAL -1000
#define sizeMAF 4  // Size of moving average filter
#define TO 3       // Seconds until timeout
#define rec 2      // How often data is sent to topic

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


rcl_publisher_t publisher_gps;

std_msgs__msg__Float32MultiArray msg;


extern "C" {
void appMain(void* arg);

extern void init_gps_wind();
extern void gps_callback(rcl_timer_t * timer, int64_t last_call_time);
}


void appMain(void* arg) {

    init_gps_wind();

    // Setup for micro-ROS

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    
    // create gps node
    rcl_node_t node_gps;// = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node_gps, "gps_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_gps, &node_gps, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/sensor/gps"));
    // create gps timer
    rcl_timer_t timer_gps;
    RCCHECK(rclc_timer_init_default(&timer_gps, &support, RCL_MS_TO_NS(100), gps_callback));
    // create gps executor
    rclc_executor_t executor_gps = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_gps, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_gps, &timer_gps));
    
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
     
    
    while (1)
    {
        rclc_executor_spin_some(&executor_gps, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    //while (1) sleep(100);

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_gps, &node_gps));
    
    RCCHECK(rcl_node_fini(&node_gps));
}
