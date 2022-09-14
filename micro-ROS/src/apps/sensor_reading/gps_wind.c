#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "components/nmea/include/nmea.h"
#include "components/protocol/include/protocol.h"
#include "components/nmea/nmea.c"
#include "components/protocol/protocol.c"
#include "driver/uart.h"

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
float angle;
float windspeed;
uint8_t* data;
uint8_t* data_wind;
char* message;
char* message2;
int calibrate = 1;
int begin_timer = 0;
int begin_timer2 = 0;
volatile int timeout = 0;
clock_t start_t, end_t;

float wind_angle, wind_speed, reading;
int errorTime = 0, errorCounter = 0;

int count_wind = 0;
int count_gps = 0;
int length = 0;

char windDir[4];
char windSpeed[6];

// Setup UART buffered IO with event queue
const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;
const uart_port_t uart_num = UART_NUM_2;

void wind_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL) { 
        // read data from the sensor
        int pos = uart_pattern_pop_pos(uart_num);
        
        if (pos != -1) {
        /* read one line(include '\n') */
        
            length = uart_read_bytes(uart_num, data_wind, pos + 1, 100 / portTICK_PERIOD_MS);
            uart_flush_input(uart_num);
            /* make sure the line is a standard string */

            data_wind[length] = '\0';
	    
	    // Code for decoding message using a while loop :(
	    const uint8_t *d = data_wind;
            uint8_t dirPos = 0;
            uint8_t speedPos = 0;
            int delimCounter = 0;
            if (*d == '$') {
        	wind_speed = 1; 
                while (*d) {
                    if (*d == ',') {
                        if (delimCounter == 0){
                        d++;
                        windDir[dirPos++] = *d;
                        d++;
                        windDir[dirPos++] = *d;
                        d++;
                        windDir[dirPos++] = *d;
                        windDir[dirPos] = '\0';
                        } else if (delimCounter == 2) {
                        d++;
                        windSpeed[speedPos++] = *d;
                        d++;
                        windSpeed[speedPos++] = *d;
                        d++;
                        windSpeed[speedPos++] = *d;
                        d++;
                        windSpeed[speedPos++] = *d;
                        d++;
                        windSpeed[speedPos++] = *d;
                        windSpeed[speedPos] = '\0';
                        break;
                        }
                        delimCounter++;
                    }
                    d++;
                    if (*d == '*') {
                        break;
                    }
                }
                wind_angle = strtof(windDir, NULL);
                wind_speed = strtof(windSpeed, NULL);
            }
		
	/* Code for decoding message using regex :)
        // Only convert up to the * sign, since that marks the end of a message
        while ((data_wind[i] != 42) && (i < 82)) {
            message2[i] = (char)data_wind[i];
            i++;
        }
                        
        wind_angle = 0;
        wind_speed = 0;  
	
        getWind(message2, &wind_angle, &wind_speed);
        */
        
        msg_wind.data.data[0] = wind_angle;
	msg_wind.data.size++;
	msg_wind.data.data[1] = wind_speed;
	msg_wind.data.size++;

	count_wind++;
	msg_wind.layout.data_offset = count_wind;

	RCSOFTCHECK(rcl_publish(&publisher_wind, &msg_wind, NULL));

	msg_wind.data.size = 0;
        
        } else {
        
            uart_flush_input(uart_num);

        } 
    }
}


void gps_callback(rcl_timer_t * timer, int64_t last_call_time) 
{
    (void) last_call_time;
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
    }
}

void init_gps_wind() {
    // variables
    i = 0;
    lon = 0;
    lat = 0;

    data = calloc(100, 4);
    data_wind = calloc(100, 4);
    message = calloc(100, 1);
    message2 = calloc(100, 1);    

    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 16, &uart_queue, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 4, 5, 18, 19));


    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(uart_num, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(uart_num, 16);
    uart_flush(uart_num); 

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
    if (message2 == NULL) {
        printf("Calloc for msg failed\n");
    }
    if (data == NULL) {
        printf("Calloc for data failed\n");
    }
    if (data_wind == NULL) {
        printf("Calloc for data failed\n");
    }

    // configure i2c
    configure_i2c_master();
}
