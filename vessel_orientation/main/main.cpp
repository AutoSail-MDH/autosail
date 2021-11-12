/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
/*
Modified by Peter Nguyen
*/

//
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_system.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "driver/uart.h"

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define PIN_SDA 21
#define PIN_CLK 22

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];      		// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 accel;
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
MPU6050 mpu;

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;

extern "C" {
	void app_main(void);
}

void task_init() {

	//Initialize I2C

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	vTaskDelay(500/portTICK_PERIOD_MS);

	//Initialize MPU and DMP

	mpu = MPU6050();
	mpu.initialize();
	RCCHECK(mpu.dmpInitialize());

	// This need to be setup individually
	//mpu.setXGyroOffset(220);
	//mpu.setYGyroOffset(76);
	//mpu.setZGyroOffset(-85);
	//mpu.setZAccelOffset(1788);

	mpu.setXGyroOffset(mpu.getXGyroOffset());
	mpu.setYGyroOffset(mpu.getYGyroOffset());
	mpu.setZGyroOffset(mpu.getZGyroOffset());
	mpu.setZAccelOffset(mpu.getZAccelOffset());
	

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateGyro(6);
	mpu.CalibrateAccel(6);

	mpu.setDMPEnabled(true);

	//Allocate message memory
	static float memory[3];
    msg.data.capacity = 3;
    msg.data.data = memory;
    msg.data.size = 0;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {

		//MPU to read and store sensor values

		mpuIntStatus = mpu.getIntStatus();
		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        mpu.resetFIFO();

	    // otherwise, check for DMP data ready interrupt frequently)
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        // read a packet from FIFO

	        mpu.getFIFOBytes(fifoBuffer, packetSize);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			//ypr[0] = ypr[0] * 180/M_PI;
			//ypr[1] = ypr[1] * 180/M_PI;
			//ypr[2] = ypr[2] * 180/M_PI;

			mpu.dmpGetAccel(&accel, fifoBuffer);
	    }

		//micro-ROS to publish to topic

		for (int32_t i = 0; i < 3; i++) {
			msg.data.data[i] = ypr[i] * 180/M_PI;
			msg.data.size++;
		}

		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

		msg.data.size = 0;
	}
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
	//Create serial UART connection using custom transport
	//Needed for micro-ROS ESP-IDF component
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
	
	//Setup for micro-ROS

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "IMU_pub", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"/position/IMU"));

	// create timer,
	rcl_timer_t timer;
	//Best result is to match with DMP refresh rate
	// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	// Now its 0x13, which means DMP is refreshed with 10Hz rate
	//100/portTICK_PERIOD_MS
	//Gyrometer: 8kHz
	//Accelerometer: 1 kHz
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

	vTaskDelay(500/portTICK_PERIOD_MS);
	task_init();

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}