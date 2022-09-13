#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>
#include <unistd.h>

#include "driver/uart.h"
#include "esp32_serial_transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_wind;
rcl_publisher_t publisher_gps;

extern void init_imu();
extern void imu_callback(void *arg);

extern "C" {
void app_main(void);

extern void init_gps_wind();
extern void gps_callback(void *arg);
extern void wind_callback(void *arg);
}

void micro_ros_task(void *arg) {
    // Setup for micro-ROS

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "multithread_node", "", &support));

    // create publishers
    RCCHECK(rclc_publisher_init_default(
        &publisher_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/position/IMU"));
    RCCHECK(rclc_publisher_init_default(
        &publisher_gps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/position/GPS"));
    RCCHECK(rclc_publisher_init_default(
        &publisher_wind, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/direction/wind"));

    // create multithreading tasks
	
    // Needs to pass the actual publisher for some reason due to it being a cpp file
    xTaskCreate(imu_callback, "imu_callback", CONFIG_MICRO_ROS_APP_STACK, &publisher_imu,
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreate(gps_callback, "gps_callback", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreate(wind_callback, "wind_callback", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);

    while (1) sleep(100);

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
    RCCHECK(rcl_publisher_fini(&publisher_gps, &node));
    RCCHECK(rcl_publisher_fini(&publisher_wind, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void) {
// Create serial UART connection using custom transport
// Needed for micro-ROS ESP-IDF component
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(true, (void *)&uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write,
                                  esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1))
        ;

    init_imu();
    init_gps_wind();

    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}