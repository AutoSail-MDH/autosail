#include "main.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "nmea.h"
#include "protocol.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Float32 message;

// variables
int i;
int c;
float lon;
float lat;
uint8_t reg_addr;
uint8_t* data;
char* msg;
i2c_ack_type_t ack;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {

        //While loop

        // read data from the sensor
        //i2c_master_write_read_device(I2C_MASTER_NUM, SLAVE_ADDR, &reg_addr, 1, data, 400, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
        //i2c_master_read(I2C_MASTER_NUM, data, 400, ack);
        i2c_read(I2C_MASTER_NUM, data, 400);

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

        // delay for easier to read prints
        vTaskDelay(50);
        //
        message.data = lon;
		RCSOFTCHECK(rcl_publish(&publisher, &message, NULL));
		//message.data++;
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "test_gps", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"topic"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	message.data = 0;

    //Setup

    // variables
    i = 0;
    c = 0;

    lon = 0;
    lat = 0;

    reg_addr = 0xFF;  // output register

    data = calloc(100, 4);
    msg = calloc(100, 1);

    if (msg == NULL) {
        printf("Calloc for msg failed\n");
    }
    if (data == NULL) {
        printf("Calloc for data failed\n");
    }

    // configure i2c
    configure_i2c_master();
    //ack = 0;
    //

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void appMain(void * arg)
{   
  // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreate(micro_ros_task, 
            "uros_task", 
            CONFIG_MICRO_ROS_APP_STACK, 
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO, 
            NULL); 
}
