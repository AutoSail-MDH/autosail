#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <autosail_message/msg/gnss_message.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "components/minmea/minmea.h"
#include "components/protocol/include/protocol.h"
#include "components/minmea/minmea.c"
#include "components/protocol/protocol.c"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_system.h"
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

#define FATAL -1000.0
#define THREE_SECONDS 3

rcl_publisher_t publisher_gnss;
autosail_message__msg__GNSSMessage gnss_msg;

// variables
int i;
float time_stamp = 0.0;
float longitude = 0.0;
float latitude = 0.0;
int gps_fix = 0.0;
char* message;
uint8_t* data;
int calibrate = 1;
int begin_timer = 0;
volatile int timeout = 0;
clock_t start_t, end_t;

int count_gnss = 0;

void gnss_callback(rcl_timer_t * timer, int64_t last_call_time) 
{
    (void) last_call_time;
    if (timer != NULL) {
        // read data from the sensor
        i2c_read(I2C_MASTER_NUM, data, 400);
        // Only convert up to the * sign, since that marks the end of a message
        while ((data[i] != 42) && (i < 82)) {
            message[i] = (char)data[i];
            i++;
        }
        switch (minmea_sentence_id(message, false)) {
            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, message)) {
                    gnss_msg.time_stamp = 1;
                    gnss_msg.longitude = 1;
                    gnss_msg.latitude = 1;
                    gnss_msg.gps_fix = 1;
                }
                else {
                    gnss_msg.time_stamp = -1.0;
                    gnss_msg.longitude = -1.0;
                    gnss_msg.latitude = -1.0;
                    gnss_msg.gps_fix = -1.0;
                }
            } break;
            case MINMEA_INVALID: {
                gnss_msg.time_stamp = -2.0;
                gnss_msg.longitude = -2.0;
                gnss_msg.latitude = -2.0;
                gnss_msg.gps_fix = -2.0;
            } break;

            default: {
                gnss_msg.time_stamp = 0.0;
                gnss_msg.longitude = 0.0;
                gnss_msg.latitude = 0.0;
                gnss_msg.gps_fix = 0;
            } break;
        }
        RCSOFTCHECK(rcl_publish(&publisher_gnss, &gnss_msg, NULL));
    }
}

void init_gnss() {
    // variables
    i = 0;
    time_stamp = 0.0;
    longitude = 0.0;
    latitude = 0.0;
    gps_fix = 0;

    gnss_msg.time_stamp = time_stamp;
    gnss_msg.longitude = longitude;
    gnss_msg.latitude = latitude;
    gnss_msg.gps_fix = gps_fix;

    data = calloc(100, 4);
    message = calloc(100, 1);

    if (data == NULL) {
        printf("Calloc for data failed\n");
    }
    
    if (message == NULL) {
        printf("Calloc for msg failed\n");
    }

    // configure i2c
    configure_i2c_master();
}
