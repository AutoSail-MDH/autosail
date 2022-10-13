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
#include "driver/uart.h"
#include "driver/gpio.h"
#include "components/nmea/include/nmea.h"


#ifdef ESP_PLATFORM
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

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
    
rcl_publisher_t publisher_wind;

extern "C" {
void appMain(void* arg);

    extern void init_wind_sensor();
    extern void wind_callback(rcl_timer_t * timer, int64_t last_call_time);
}

void appMain(void* arg) {

    init_wind_sensor();

    // Setup for micro-ROS

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create wind node
    rcl_node_t node_wind;// = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node_wind, "wind_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_wind, &node_wind, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/sensor/wind"));
    // create wind timer
    rcl_timer_t timer_wind;
    RCCHECK(rclc_timer_init_default(&timer_wind, &support, RCL_MS_TO_NS(100), wind_callback));
    // create wind executor
    rclc_executor_t executor_wind = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_wind, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_wind, &timer_wind));
    
    
    while (1)
    {
        rclc_executor_spin_some(&executor_wind, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    //while (1) sleep(100);

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_wind, &node_wind));
    
    RCCHECK(rcl_node_fini(&node_wind));
}
