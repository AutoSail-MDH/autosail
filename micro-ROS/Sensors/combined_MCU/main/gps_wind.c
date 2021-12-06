#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "include/nmea.h"
#include "nmea.c"
#include "include/protocol.h"
#include "protocol.c"
#include "driver/adc.h"
#include "driver/gpio.h"

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
#define FATAL -1000.0
#define THREE_SECONDS 3
#define NO_OF_SAMPLES 64

rcl_publisher_t publisher_wind;
rcl_publisher_t publisher_gps;
std_msgs__msg__Float32MultiArray msg_gps;
std_msgs__msg__Float32MultiArray msg_wind;

// variables
int i;
float lon;
float lat;
uint8_t* data;
char* message;
int calibrate = 1;
int begin_timer = 0;
volatile int timeout = 0;
clock_t start_t, end_t;

float windDir, direction, reading;
static const adc_channel_t channel = ADC1_CHANNEL_0;
// static const adc_atten_t atten = ADC_ATTEN_11db;

void wind_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Reset reading between
        reading = 0;

        // Multisampling, takes an amount of samples and averages them
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            reading += adc1_get_raw((adc1_channel_t)channel);
        }

        // Average readings
        windDir = reading / NO_OF_SAMPLES;
        // Convert to radians
        direction = windDir / 12.3;

        // Adjust for starting position (wait for hardware team to modify)
        direction += 32;

        // Bound to [0, 360]
        if (direction > 360) {
            direction = direction - 360;
        }

        msg_wind.data.data[0] = windDir;
        msg_wind.data.size++;
        msg_wind.data.data[1] = direction;
        msg_wind.data.size++;

        RCCHECK(rcl_publish(&publisher_wind, &msg_wind, NULL));

        msg_wind.data.size = 0;
    }
}

void gps_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // read data from the sensor
        i2c_read(I2C_MASTER_NUM, data, 400);

        // convert the data to char
        i = 0;
        lon = 0;
        lat = 0;

        // Only convert up to the * sign, since that marks the end of a message
        while ((data[i] != 42) && (i < 82)) {
            message[i] = (char)data[i];
            i++;
        }

        // get the GPS position
        if (!getPos(message, &lat, &lon)) {
            if (calibrate == 0) {
                begin_timer = 1;
            }
            // printf("No data avalible for %d readings\n", c);
        } else {
            start_t = clock();
            calibrate = 0;
            begin_timer = 0;
            // printf("Lat: %.4f : Long: %.4f\n", lat, lon);
        }

        //
        if (begin_timer) {
            end_t = clock();
            if (((end_t - start_t) / CLOCKS_PER_SEC) >= THREE_SECONDS) {
                timeout = 1;
                msg_gps.data.data[0] = FATAL;
                msg_gps.data.size++;
                msg_gps.data.data[1] = FATAL;
                msg_gps.data.size++;
            }
        }

        if ((lon != 0 && lat != 0) && timeout == 0) {
            msg_gps.data.data[0] = lat;
            msg_gps.data.size++;
            msg_gps.data.data[1] = lon;
            msg_gps.data.size++;
        }

        RCSOFTCHECK(rcl_publish(&publisher_gps, &msg_gps, NULL));
        msg_gps.data.size = 0;
    }
    // delay for easier to read prints
    // vTaskDelay(25);
}

void init_gps_wind(rcl_allocator_t* allocator, rclc_support_t* support, rclc_executor_t* executor) {

    // create node
    rcl_node_t gps_node;
    rcl_node_t wind_node;
    RCCHECK(rclc_node_init_default(&gps_node, "GPS_pub", "", support));
    RCCHECK(rclc_node_init_default(&wind_node, "wind_node", "", support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher_gps, &gps_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/position/GPS"));
    RCCHECK(rclc_publisher_init_default(
        &publisher_wind, &wind_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/direction/wind"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(&timer, support, RCL_MS_TO_NS(timer_timeout), gps_callback));

    // create timer,
    rcl_timer_t timer_wind;
    RCCHECK(rclc_timer_init_default(&timer_wind, support, RCL_MS_TO_NS(timer_timeout), wind_callback));

    // create executor
    RCCHECK(rclc_executor_add_timer(executor, &timer));
    RCCHECK(rclc_executor_add_timer(executor, &timer_wind));
    // RCCHECK(rclc_executor_add_subscription(&executor, &publisher_gps, &msg_gps, &gps_callback, ON_NEW_DATA));
    // RCCHECK(rclc_executor_add_subscription(&executor, &publisher_wind, &msg_wind, &wind_callback, ON_NEW_DATA));

    // variables
    i = 0;

    lon = 0;
    lat = 0;

    data = calloc(100, 4);
    message = calloc(100, 1);

    // msg setup
    static float memory_gps[2];
    msg_gps.data.capacity = 2;
    msg_gps.data.data = memory_gps;
    msg_gps.data.size = 0;

    static float memory_wind[2];
    msg_wind.data.capacity = 2;
    msg_wind.data.data = memory_wind;
    msg_wind.data.size = 0;

    if (message == NULL) {
        printf("Calloc for msg failed\n");
    }
    if (data == NULL) {
        printf("Calloc for data failed\n");
    }

    // configure i2c
    configure_i2c_master();

    while (1) {
    	rclc_executor_spin_some(executor, RCL_MS_TO_NS(50));
	usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_gps, &gps_node));
    RCCHECK(rcl_publisher_fini(&publisher_wind, &wind_node));
    RCCHECK(rcl_node_fini(&gps_node));
    RCCHECK(rcl_node_fini(&wind_node));
}
