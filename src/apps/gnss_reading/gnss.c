#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <autosail_message/msg/gnss_message.h>
#include <autosail_message/msg/nmea.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "components/I2C/include/devI2C.h"
#include "components/nmea/include/nmea_parser.h"
#include "components/regex/include/regex_parser.h"
#include "components/I2C/devI2C.c"
#include "components/nmea/nmea_parser.c"
#include "components/regex/regex_parser.c"

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
autosail_message__msg__NMEA nmea_msg;

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
        //i2c_read(I2C_MASTER_NUM, data, 400);
        i = 0;
        i2c_master_read_slave_reg(I2C_MASTER_NUM, data, 400);
        // Only convert up to the * sign, since that marks the end of a message
        while ((data[i] != 42) && (i < 82)) {
            message[i] = (char)data[i];
            i++;
        }

        get_position(message, &time_stamp, &latitude, &longitude, &gps_fix);
        gnss_msg.time_stamp = time_stamp;
        gnss_msg.position.latitude = latitude;
        gnss_msg.position.longitude = longitude;
        gnss_msg.gps_fix = gps_fix;
        
        /*
        gps_fix = rget_position(message, &timestamp, &latitude, &longitude);
        gnss_msg.time_stamp = timestamp;
        gnss_msg.position.latitude = latitude;
        gnss_msg.position.longitude = longitude;
        gnss_msg.gps_fix = gps_fix;
        */
        
        /*
        nmea_msg.one = message[0];
        nmea_msg.two = message[1];
        nmea_msg.three = message[2];
        nmea_msg.four = message[3];
        nmea_msg.five = message[4];
        nmea_msg.six = message[5];
        nmea_msg.seven = message[16];
        nmea_msg.eight = message[17];
        nmea_msg.nine = message[18];
        nmea_msg.ten = message[19];
        nmea_msg.eleven = message[20];
        nmea_msg.twelve = message[21];
        nmea_msg.thirteen = message[22];
        nmea_msg.fourteen = message[23];
        nmea_msg.fiveteen = message[24];
        nmea_msg.sixteen = message[25];
        nmea_msg.seventeen = message[26];
        nmea_msg.eighteen = message[27];
        nmea_msg.nineteen = message[28];
        nmea_msg.twenty = message[29];
        nmea_msg.twentyone = message[30];
        nmea_msg.twentytwo = message[31];
        nmea_msg.twentythree = message[32];
        nmea_msg.twentyfour = message[33];
        nmea_msg.twentyfive = message[34];
        nmea_msg.twentysix = message[35];
        */

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
    gnss_msg.position.latitude = latitude;
    gnss_msg.position.longitude = longitude;
    gnss_msg.gps_fix = gps_fix;
    
    /*
    nmea_msg.one = 0;
    nmea_msg.two = 0;
    nmea_msg.three = 0;
    nmea_msg.four = 0;
    nmea_msg.five = 0;
    nmea_msg.six = 0;
    nmea_msg.seven = 0;
    nmea_msg.eight = 0;
    nmea_msg.nine = 0;
    nmea_msg.ten = 0;
    nmea_msg.eleven = 0;
    nmea_msg.twelve = 0;
    nmea_msg.thirteen = 0;
    nmea_msg.fourteen = 0;
    nmea_msg.fiveteen = 0;
    nmea_msg.sixteen = 0;
    nmea_msg.seventeen = 0;
    nmea_msg.eighteen = 0;
    nmea_msg.nineteen = 0;
    nmea_msg.twenty = 0;
    nmea_msg.twentyone = 0;
    nmea_msg.twentytwo = 0;
    nmea_msg.twentythree = 0;
    nmea_msg.twentyfour = 0;
    nmea_msg.twentyfive = 0;
    nmea_msg.twentysix = 0;
    */

    data = calloc(100, 4);
    message = calloc(100, 1);

    if (data == NULL) {
        printf("Calloc for data failed\n");
    }
    
    if (message == NULL) {
        printf("Calloc for msg failed\n");
    }

    // configure i2c
    i2c_master_init();
}
