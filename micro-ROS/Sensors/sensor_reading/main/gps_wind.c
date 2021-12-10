#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "include/nmea.h"
#include "nmea.c"
#include "include/protocol.h"
#include "protocol.c"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);esp_restart();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define INIT 94
#define FATAL -1000.0
#define THREE_SECONDS 3
#define NO_OF_SAMPLES 64

rcl_publisher_t publisher_wind;
rcl_publisher_t publisher_gps;
std_msgs__msg__Float32MultiArray msg_wind;
std_msgs__msg__Float32MultiArray msg_gps;

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
int errorTime = 0, errorCounter = 0;
static const adc_channel_t channel = ADC1_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_11db;

int count_wind = 0;
int count_gps = 0;

void wind_callback(void * arg) {

    TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount();

    while (1) {
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

        errorTime++;
        if (windDir != -1)
            errorTime = 0;
        else
            if(errorTime == 1)
                errorCounter++;
        
        if (errorTime >= 100 || errorCounter >= 4) {
		    msg_wind.data.data[0] = FATAL;
		    msg_wind.data.size++;
	    } else {
            msg_wind.data.data[0] = windDir;
            msg_wind.data.size++;
            msg_wind.data.data[1] = direction;
            msg_wind.data.size++;
        }

        count_wind++;
        msg_wind.layout.data_offset = count_wind;

        RCSOFTCHECK(rcl_publish(&publisher_wind, &msg_wind, NULL));

        msg_wind.data.size = 0;

        //usleep(100000);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void gps_callback(void * arg) {

    TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount();

    while (1) {
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
        } else {
            msg_gps.data.data[0] = 0.0;
            msg_gps.data.size++;
            msg_gps.data.data[1] = 0.0;
            msg_gps.data.size++;
        }

        count_gps++;
        msg_gps.layout.data_offset = count_gps;

        RCSOFTCHECK(rcl_publish(&publisher_gps, &msg_gps, NULL));
        msg_gps.data.size = 0;

        //usleep(100000);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void init_gps_wind() {

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

    adc1_config_channel_atten(channel, atten);
}