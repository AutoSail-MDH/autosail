#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>
#include <unistd.h>

#include "main.h"
#include "nmea.h"
#include "protocol.h"

#ifdef ESP_PLATFORM
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

/*TO DO*/
/*Check the includes so that we can find them*/
/*Check so that the structure, folders and files, are correct*/
/*Check that everything we need for micro ros is there*/
/*Check the node and make sure that we have one we want*/
/*Check the topic and where we publish*/
/*Check the buffers, the buffer we publish, msg_pub, and that the right data is in it when we publish. Reset after?*/
/*Test the code*/

rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;
std_msgs__msg__Float32MultiArray msg_pub;  // kommer inte ihåg om peter ville ha float32 eller 64, gissade på 32

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        i2c_master_write_read_device(I2C_MASTER_NUM, SLAVE_ADDR, &reg_addr, 1, data, 400,
                                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

        // convert the data to char
        i = 0;

        lon = 0;
        lat = 0;
        // Only convert up to the * sign, since that marks the end of a message
        while ((data[i] != 42) && (i < 100)) {
            msg[i] = (char)data[i];
            i++;
        }

        // get the GPS position
        if (!getPos(msg, &lat, &lon)) {
            c++;
            printf("No data avalible for %d readings\n", c);
        } else {
            c = 0;
            printf("Lat: %.4f : Long: %.4f\n", lat, lon);
        }

        // put position in array? in msg_pub

        RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
        msg_pub.data++;
    }
}

void appMain(void* arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "freertos_int32_publisher", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(&publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg_pub, Float32),  // msg, Int32
                                        "freertos_int32_publisher"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg_pub.data = 0;  // msg.data

    int i = 0;
    int c = 0;

    float lon = 0;
    float lat = 0;

    uint8_t reg_addr = 0xFF;  // output register

    uint8_t* data = calloc(100, 4);
    char* msg = calloc(100, 1);

    if (msg == NULL) {
        printf("Calloc for msg failed\n");
    }
    if (data == NULL) {
        printf("Calloc for data failed\n");
    }

    // configure i2c
    configure_i2c_master();

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node))
    RCCHECK(rcl_node_fini(&node))

    vTaskDelete(NULL);
}
