#include <esp_err.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "components/BNO055ESP32/BNO055ESP32.cpp"
#include "components/I2Cdev/I2Cdev.cpp"
#include "components/BNO055ESP32/BNO055ESP32.h"
#include "components/I2Cdev/I2Cdev.h"

#include <std_msgs/msg/int32.h>

#ifdef ESP_PLATFORM
#include "driver/i2c.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define PIN_SDA 21
#define PIN_CLK 22

#define FATAL -1000
#define sizeMAF 4  // Size of moving average filter
#define TO 3       // Seconds until timeout
#define rec 2      // How often data is sent to topic

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

bno055_quaternion_t q;
bno055_vector_t gravity;
bno055_vector_t linAccel;
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float accel[3];         // [x, y, z]		linear acceleration
float yprBuffer[3][sizeMAF];
float accelBuffer[3][sizeMAF];
bno055_interrupts_status_t mpuIntStatus;      // holds actual interrupt status byte from MPU
uint32_t count = 0;
BNO055* bno = NULL;
struct timeval currTime;
struct timeval prevTime;

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;

rcl_publisher_t publisher_2;
std_msgs__msg__Int32 msg_2;

extern "C" {
void appMain(void* arg);
}

void task_init() {
    // Initialize I2C

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)PIN_SDA;
    conf.scl_io_num = (gpio_num_t)PIN_CLK;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_set_timeout(I2C_NUM_0, 30000);

    //to use i²C leave the following line active
    bno = new BNO055((i2c_port_t)I2C_NUM_0, 0x28); // BNO055 I2C Addr can be 0x28 or 0x29 (depends on your hardware)


    // bno055_offsets_t storedOffsets;
    // storedOffsets.accelOffsetX = 29;
    // storedOffsets.accelOffsetY = 24;
    // storedOffsets.accelOffsetZ = 16;
    // storedOffsets.magOffsetX = -243;
    // storedOffsets.magOffsetY = -420;
    // storedOffsets.magOffsetZ = -131;
    // storedOffsets.gyroOffsetX = 1;
    // storedOffsets.gyroOffsetY = -1;
    // storedOffsets.gyroOffsetZ = 0;
    // storedOffsets.accelRadius = 0;
    // storedOffsets.magRadius = 662;


    bno->begin();  // BNO055 is in CONFIG_MODE until it is changed
    bno->enableExternalCrystal();
    // bno.setSensorOffsets(storedOffsets);
    // bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
    /* you can specify a PoWeRMode using:
            - setPwrModeNormal(); (Default on startup)
            - setPwrModeLowPower();
            - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
    */
    bno->setOprModeNdof();

    // Allocate message memory
    static float memory[6];
    msg.data.capacity = 6;
    msg.data.data = memory;
    msg.data.size = 0;

    gettimeofday(&prevTime, NULL);
}

void GetHeading(float *data, bno055_quaternion_t *q, bno055_vector_t *gravity) {
        // yaw: (about Z axis)
        data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
        // pitch: (nose up/down, about Y axis)
        data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
        // roll: (tilt left/right, about X axis)
        data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
        return;
}

void moving_average_filter(float avg[], float buffer[3][sizeMAF]) {
    for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < sizeMAF; j++) avg[i] = avg[i] + buffer[i][j];

    for (int32_t i = 0; i < 3; i++) avg[i] = avg[i] / sizeMAF;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // MPU to read and store sensor values

        //mpuIntStatus = bno.getInterruptsStatus();
        //fifoCount = mpu.getFIFOCount();  // get current FIFO count

        gettimeofday(&currTime, NULL);

        q = bno->getQuaternion();         // [w, x, y, z]         quaternion container
        gravity = bno->getVectorGravity(); // [x, y, z]           gravity vector
        linAccel = bno->getVectorLinearAccel(); // [x, y, z]	    linear acceleration 
        

        // Get heading
        GetHeading(ypr, &q, &gravity);
        accel[0] = linAccel.x;
        accel[1] = linAccel.y;
        accel[2] = linAccel.z;
        
        
        //bno->clearInterruptPin();  // don't forget to place this.

        // Fill buffer by replacing oldest value
        for (int32_t i = 0; i < 3; i++) yprBuffer[i][count % sizeMAF] = ypr[i];	// yaw pitch roll
        
        for (int32_t i = 0; i < 3; i++) accelBuffer[i][count % sizeMAF] = accel[i];	// linear accel

        if ((count % rec == 0) && (count >= sizeMAF)) {
            moving_average_filter(ypr, yprBuffer);
            moving_average_filter(accel, accelBuffer);

            // micro-ROS to publish to topic
            for (int32_t i = 0; i < 3; i++) {
                msg.data.data[i] = ypr[i] * 180 / M_PI;
                msg.data.size++;
                msg.data.data[i+3] = accel[i];
                msg.data.size++;
            }
            msg.layout.data_offset = count / rec;

            RCCHECK(rcl_publish(&publisher, &msg, NULL));
            vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
        }
        
        prevTime = currTime;
        count++;

        msg.data.size = 0;
    }
}

void timer_callback_2(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCCHECK(rcl_publish(&publisher_2, &msg_2, NULL));
        msg_2.data++;
    }
}


void appMain(void* arg) {
    // Setup for micro-ROS

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "IMU_pub", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/position/IMU"));

    // create timer,
    rcl_timer_t timer;
    // Best result is to match with DMP refresh rate
    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
    // Now its 0x13, which means DMP is refreshed with 10Hz rate
    // Gyrometer: 8kHz
    // Accelerometer: 1 kHz
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    task_init();




    rcl_node_t node_2;
    RCCHECK(rclc_node_init_default(&node_2, "IMUnt", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_2, &node_2, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/testing"));
    rcl_timer_t timer_2;
    RCCHECK(rclc_timer_init_default(&timer_2, &support, RCL_MS_TO_NS(timer_timeout), timer_callback_2));
    rclc_executor_t executor_2;
    RCCHECK(rclc_executor_init(&executor_2, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_2, &timer_2));




    vTaskDelay(500 / portTICK_PERIOD_MS);

    msg_2.data = 0;
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor_2, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_publisher_fini(&publisher_2, &node));
    RCCHECK(rcl_node_fini(&node_2));

    vTaskDelete(NULL);
}