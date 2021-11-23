#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "driver/gpio.h"
#include "driver/adc.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);esp_restart();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define NO_OF_SAMPLES 64

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;

float windDir, direction, reading;

//Set channel pin and attenuation (anything high enough produces same results for attenuation)
static const adc_channel_t channel = ADC1_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_11db;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		//Reset reading between 
        reading = 0;

        //Multisampling, takes an amount of samples and averages them
        for(int i = 0; i < NO_OF_SAMPLES; i++){

            reading += adc1_get_raw((adc1_channel_t) channel);

        }

        //Average readings
        windDir = reading / NO_OF_SAMPLES;
        //Convert to radians
        direction = windDir / 12.3;

        //Adjust for starting position (wait for hardware team to modify)
        direction += 32;

        //Bound to [0, 360]
        if(direction > 360){

            direction = direction - 360;

        }

		msg.data.data[0] = windDir;
		msg.data.size++;
		msg.data.data[1] = direction;
		msg.data.size++;

		RCCHECK(rcl_publish(&publisher, &msg, NULL));

		msg.data.size = 0;
	}
}

void micro_ros_task(void * arg)
{
	while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1));
	
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "wind_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"direction/wind"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 100;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	
	//
	//Configure attenuation
	adc1_config_channel_atten(channel, atten);

	static float memory[2];
	msg.data.capacity = 2;
	msg.data.data = memory;
	msg.data.size = 0;
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

static size_t uart_port = 0; //UART_NUM_0;

void app_main(void)
{   
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreate(micro_ros_task, 
            "uros_task", 
            CONFIG_MICRO_ROS_APP_STACK, 
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO, 
            NULL); 
}
