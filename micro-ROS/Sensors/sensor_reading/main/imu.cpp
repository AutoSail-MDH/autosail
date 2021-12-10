#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"

#define PIN_SDA 21
#define PIN_CLK 22

#define FATAL	-1000
#define sizeMAF 4	//Size of moving average filter
#define TO		3	//Seconds until timeout
#define rec		2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);esp_restart();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprBuffer[3][sizeMAF];
VectorInt16 gyroInt16;
float gyro[3];			// []
float gyroBuffer[3][sizeMAF];
VectorInt16 accelInt16;
float accel[3];			// []
float accelBuffer[3][sizeMAF];
uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
MPU6050 mpu;

uint32_t count = 0;
struct timeval currTime;
struct timeval prevTime;

std_msgs__msg__Float32MultiArray msg;

void init_imu() {

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

	mpu.setXGyroOffset(mpu.getXGyroOffset());
	mpu.setYGyroOffset(mpu.getYGyroOffset());
	mpu.setZGyroOffset(mpu.getZGyroOffset());
	
	mpu.setXAccelOffset(mpu.getXAccelOffset());
	mpu.setYAccelOffset(mpu.getYAccelOffset());
	mpu.setZAccelOffset(mpu.getZAccelOffset());

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateGyro(6);
	mpu.CalibrateAccel(6);

	mpu.setZAccelOffset(mpu.getZAccelOffset() + 4.7);

	mpu.setDMPEnabled(true);

	//Allocate message memory
	static float memory[9];
	msg.data.capacity = 9;
	msg.data.data = memory;
	msg.data.size = 0;

	gettimeofday(&prevTime,NULL);
}

void moving_average_filter(float avg[], float buffer[3][sizeMAF])
{
	for (int32_t i = 0; i < 3; i++)
		for (int32_t j = 0; j < sizeMAF; j++)
			avg[i] = avg[i] + buffer[i][j];
	
	for (int32_t i = 0; i < 3; i++)
		avg[i] = avg[i] / sizeMAF;
}

void imu_callback(void * arg)
{
	rcl_publisher_t* publisher = (rcl_publisher_t*) arg;

	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 10;
	xLastWakeTime = xTaskGetTickCount();

	while (1) {

		//MPU to read and store sensor values

		mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount(); // get current FIFO count

		gettimeofday(&currTime,NULL);

		float timeDiff = abs((currTime.tv_sec - prevTime.tv_sec));

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	        mpu.resetFIFO(); // reset so we can continue cleanly

	    // otherwise, check for DMP data ready interrupt frequently)
	    else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while ((fifoCount < packetSize) && (timeDiff < TO)) {
				fifoCount = mpu.getFIFOCount();

				//To not get stuck
				gettimeofday(&currTime,NULL);
				timeDiff = abs((currTime.tv_sec - prevTime.tv_sec));
			}

	        // read a packet from FIFO

	        mpu.getFIFOBytes(fifoBuffer, packetSize);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);

			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			VectorInt16 temp;
			mpu.dmpGetAccel(&accelInt16, fifoBuffer);
			memcpy(&temp, &accelInt16, sizeof(temp));
			mpu.dmpGetLinearAccel(&accelInt16, &temp, &gravity);
			memcpy(&temp, &accelInt16, sizeof(temp));
			mpu.dmpGetLinearAccelInWorld(&accelInt16, &temp, &q);

			mpu.dmpGetGyro(&gyroInt16, fifoBuffer);

			accel[0] = (float)accelInt16.z;
			accel[1] = (float)accelInt16.y;
			accel[2] = (float)accelInt16.x;

			gyro[0] = (float)gyroInt16.z;
			gyro[1] = (float)gyroInt16.y;
			gyro[2] = (float)gyroInt16.x;

			//Fill buffer by replacing oldest value
			for (int32_t i = 0; i < 3; i++) {
				yprBuffer[i][count % sizeMAF] = ypr[i];
				accelBuffer[i][count % sizeMAF] = accel[i];
				gyroBuffer[i][count % sizeMAF] = gyro[i];
			}

			//Every other sample and only if buffer is full
			if ((count % rec == 0) && (count >= sizeMAF)) {
				moving_average_filter(ypr, yprBuffer);
				moving_average_filter(accel, accelBuffer);
				moving_average_filter(gyro, gyroBuffer);

				//micro-ROS to publish to topic

				for (int32_t i = 0; i < 3; i++) {
					msg.data.data[i] = ypr[i] * 180/M_PI;
					msg.data.data[i+3] = accel[i];
					msg.data.data[i+6] = gyro[i];
					msg.data.size += 3;
				}
				msg.layout.data_offset = count/rec; //xLastWakeTime/portTICK_PERIOD_MS

				RCCHECK(rcl_publish(publisher, &msg, NULL));
			}
			prevTime = currTime;
			count++;
	    }
		
		if ((timeDiff > TO) && (count > sizeMAF)) {
			msg.data.data[0] = FATAL;
			msg.data.size++;
			RCCHECK(rcl_publish(publisher, &msg, NULL));
		}

		msg.data.size = 0;

		//Best result is to match with DMP refresh rate
		// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		// Now its 0x13, which means DMP is refreshed with 10Hz rate
		//100/portTICK_PERIOD_MS
		//Gyrometer: 8kHz
		//Accelerometer: 1 kHz
		//usleep(100000);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}