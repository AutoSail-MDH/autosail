#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <autosail_message/msg/gnss_message.h>
#include <autosail_message/msg/nmea.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

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


rcl_publisher_t publisher_gnss;


extern "C" {
void appMain(void* arg);

extern void init_gnss();
extern void gnss_callback(rcl_timer_t * timer, int64_t last_call_time);
}


void appMain(void* arg) {

    init_gnss();

    // Setup for micro-ROS

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    
    // create gnss node
    rcl_node_t node_gnss;// = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node_gnss, "gnss_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_gnss, &node_gnss, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, NMEA), "/sensor/gnss"));
    // create gnss timer
    rcl_timer_t timer_gnss;
    RCCHECK(rclc_timer_init_default(&timer_gnss, &support, RCL_MS_TO_NS(100), gnss_callback));
    // create gnss executor
    rclc_executor_t executor_gnss = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_gnss, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_gnss, &timer_gnss));
    
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
     
    
    while (1)
    {
        rclc_executor_spin_some(&executor_gnss, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    //while (1) sleep(100);

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_gnss, &node_gnss));
    
    RCCHECK(rcl_node_fini(&node_gnss));
}
