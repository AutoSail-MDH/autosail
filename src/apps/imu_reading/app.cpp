#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <autosail_message/msg/imu_message.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "components/BNO055ESP32/BNO055ESP32.cpp"
#include "components/BNO055ESP32/BNO055ESP32.h"

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

//Error check function. If error found restart microcontroller
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

rcl_publisher_t publisher_imu;
autosail_message__msg__IMUMessage msg_imu;// custom message

bno055_quaternion_t q;
bno055_vector_t gravity;
bno055_vector_t linear_acceleration;
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float acceleration[3];         // [x, y, z]		linear acceleration
float ypr_buffer[3][sizeMAF];
float acceleration_buffer[3][sizeMAF];
uint32_t count = 0;
BNO055* bno = NULL;
struct timeval current_time;
struct timeval previous_time;

extern "C" {
void appMain(void* arg);
}

void InitImu() {
    // Initialize I2C

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)PIN_SDA;
    conf.scl_io_num = (gpio_num_t)PIN_CLK;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 30000));

    //to use i²C leave the following line active
    bno = new BNO055((i2c_port_t)I2C_NUM_0, 0x28); // BNO055 I2C Addr can be 0x28 or 0x29 (depends on your hardware)

    bno->begin();  // BNO055 is in CONFIG_MODE until it is changed
    bno->enableExternalCrystal();

    gettimeofday(&previous_time, NULL);
    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Update the sensor offsets with previously saved values.
    // These offsets were fetched by using the "imu_calibration" app.
    bno055_offsets_t manualOffsets;
    manualOffsets.accelOffsetX = -25;
    manualOffsets.accelOffsetY = -32;
    manualOffsets.accelOffsetZ = -40;
    manualOffsets.accelRadius = 1000.0;
    manualOffsets.gyroOffsetX = 1;
    manualOffsets.gyroOffsetY = -3;
    manualOffsets.gyroOffsetZ = 1;
    manualOffsets.magOffsetX = -95;
    manualOffsets.magOffsetY = 432;
    manualOffsets.magOffsetZ = -234;
    manualOffsets.magRadius = 884;

    //set offsets
    bno->setSensorOffsets(manualOffsets);

    bno->setOprModeNdof();
    
}

void GetHeading(float *data, bno055_quaternion_t *q, bno055_vector_t *gravity) {
        // yaw: (about Z axis)
        data[0] = atan2(2*(q->x*q->y + q->z*q->w) , q->x*q->x - q->y*q->y - q->z*q->z + q->w*q->w);

        // pitch: (nose up/down, about Y axis)
        data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z)); 

        // roll: (tilt left/right, about X axis)
        data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z)); 

        return;
}

// GIVES INCORRECT VALUES!
void MovingAverageFilter(float avg[], float buffer[3][sizeMAF]) {
    for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < sizeMAF; j++) avg[i] = avg[i] + buffer[i][j];

    for (int32_t i = 0; i < 3; i++) avg[i] = avg[i] / sizeMAF;
}

void ImuCallback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {

        gettimeofday(&current_time, NULL);

        q = bno->getQuaternion();         // [w, x, y, z]         quaternion container
        gravity = bno->getVectorGravity(); // [x, y, z]            gravity vector
        linear_acceleration = bno->getVectorLinearAccel(); // [x, y, z]	    linear acceleration 

        // Get heading
        GetHeading(ypr, &q, &gravity);
        acceleration[0] = linear_acceleration.x;
        acceleration[1] = linear_acceleration.y;
        acceleration[2] = linear_acceleration.z;
        
        // Fill buffer by replacing oldest value
        for (int32_t i = 0; i < 3; i++) ypr_buffer[i][count % sizeMAF] = ypr[i]; // yaw pitch roll

        for (int32_t i = 0; i < 3; i++) acceleration_buffer[i][count % sizeMAF] = acceleration[i];	// linear accel

        if ((count%rec == 0) && (count >= sizeMAF)) {
            MovingAverageFilter(acceleration, acceleration_buffer);

            // fill message with gyro and accel values.
            msg_imu.yaw = ypr[0] * 180 / M_PI;
            msg_imu.pitch = ypr[1] * 180 / M_PI;
            msg_imu.roll = ypr[2] * 180 / M_PI;
            msg_imu.linear_acceleration_x = acceleration[0];
            msg_imu.linear_acceleration_y = acceleration[1];
            msg_imu.linear_acceleration_z = acceleration[2];

            // micro-ROS to publish to topic
            RCCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
            vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
        }
        
        previous_time = current_time;
        count++;
    }
}
void appMain(void* arg) {

    InitImu();

    // Setup for micro-ROS

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
       
    // create imu node
    rcl_node_t node_imu;
    RCCHECK(rclc_node_init_default(&node_imu, "imu_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_imu, &node_imu, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, IMUMessage), "/sensor/imu"));
    // create imu timer
    rcl_timer_t timer_imu;
    RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(100), ImuCallback));
    // create imu executor
    rclc_executor_t executor_imu = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_imu, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_imu, &timer_imu));
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    while (1)
    {
        rclc_executor_spin_some(&executor_imu, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    //while (1) sleep(100);

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node_imu));
    
    // destroy node
    RCCHECK(rcl_node_fini(&node_imu));
}
