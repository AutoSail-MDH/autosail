#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>
#include <unistd.h>

#include "components/nmea/include/nmea.h"
#include "components/nmea/nmea.c"
#include "components/protocol/include/protocol.h"
#include "components/protocol/protocol.c"

#ifdef ESP_PLATFORM
#include "driver/i2c.h"
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
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK)) {                                                     \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

#define INIT 94

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;

// variables
int i;
int c;
float lon;
float lat;
uint8_t reg_addr;
uint8_t* data;
char* message;
i2c_ack_type_t ack;

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // read data from the sensor
        i2c_read(I2C_MASTER_NUM, data, 400);

        // convert the data to char
        i = 0;
        lon = 0;
        lat = 0;

        // Only convert up to the * sign, since that marks the end of a message
        while ((data[i] != 42) && (i < 100)) {
            message[i] = (char)data[i];
            i++;
        }

        // get the GPS position
        if (!getPos(message, &lat, &lon)) {
            c++;
            // printf("No data avalible for %d readings\n", c);
        } else {
            c = 0;
            // printf("Lat: %.4f : Long: %.4f\n", lat, lon);
        }

        if (lon != 0 || lat != 0) {
            msg.data.data[0] = lat;
            msg.data.size++;
            msg.data.data[1] = lon;
            msg.data.size++;
            RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
            msg.data.size = 0;
        }
        // delay for easier to read prints
        // vTaskDelay(25);
    }
}

void appMain(void* arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "GPS_pub", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/position/GPS"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // variables
    i = 0;
    c = 0;

    lon = 0;
    lat = 0;

    data = calloc(100, 4);
    message = calloc(100, 1);

    // msg setup
    static float memory[2];
    msg.data.capacity = 2;
    msg.data.data = memory;
    msg.data.size = 0;

    if (message == NULL) {
        printf("Calloc for msg failed\n");
    }
    if (data == NULL) {
        printf("Calloc for data failed\n");
    }

    // configure i2c
    configure_i2c_master();

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node))
    RCCHECK(rcl_node_fini(&node))
}
