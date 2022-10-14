#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "components/BNO055ESP32/BNO055ESP32.cpp"
#include "components/I2Cdev/I2Cdev.cpp"
#include "components/BNO055ESP32/BNO055ESP32.h"
#include "components/I2Cdev/I2Cdev.h"


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
std_msgs__msg__Float32MultiArray msg;


extern "C" {
void appMain(void* arg);
}

void init_imu() {
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

void imu_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {

        gettimeofday(&currTime, NULL);

        q = bno->getQuaternion();         // [w, x, y, z]         quaternion container
        gravity = bno->getVectorGravity(); // [x, y, z]            gravity vector
        linAccel = bno->getVectorLinearAccel(); // [x, y, z]	    linear acceleration 

        // Get heading
        GetHeading(ypr, &q, &gravity);
        accel[0] = linAccel.x;
        accel[1] = linAccel.y;
        accel[2] = linAccel.z;
        
        // Fill buffer by replacing oldest value
        for (int32_t i = 0; i < 3; i++) yprBuffer[i][count % sizeMAF] = ypr[i]; // yaw pitch roll

        for (int32_t i = 0; i < 3; i++) accelBuffer[i][count % sizeMAF] = accel[i];	// linear accel

        if ((count % rec == 0) && (count >= sizeMAF)) {
                moving_average_filter(ypr, yprBuffer);
                moving_average_filter(accel, accelBuffer);

            // micro-ROS to publish to topic
            for (int32_t i = 0; i < 3; i++) {
                msg.data.data[i] = ypr[i] * 180 / M_PI;
                msg.data.data[i+3] = accel[i];
                msg.data.size += 2;
            }
            msg.layout.data_offset = count / rec;

            RCCHECK(rcl_publish(&publisher_imu, &msg, NULL));
            vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
        }
        
        prevTime = currTime;
        count++;

        msg.data.size = 0;
    }
}
void appMain(void* arg) {

    init_imu();

    // Setup for micro-ROS

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
       
    // create imu node
    rcl_node_t node_imu;
    RCCHECK(rclc_node_init_default(&node_imu, "imu_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_imu, &node_imu, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/sensor/imu"));
    // create imu timer
    rcl_timer_t timer_imu;
    RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(100), imu_callback));
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
