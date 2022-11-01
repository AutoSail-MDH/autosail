#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <autosail_message/msg/imu_calibration_message.h>
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
autosail_message__msg__IMUCalibrationMessage msg_imu;// custom message

bno055_calibration_t cal;// calibration values
bno055_offsets_t offsets;// saved offsets 
bool isCalibrated;// if IMU is calibrated or not

bno055_quaternion_t q;
bno055_vector_t gravity;
bno055_vector_t linAccel;
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint32_t count = 0;
BNO055* bno = NULL;

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

    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Update the sensor offsets with previously saved values. Do this by using setSensorOffsets()
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

    bno->setSensorOffsets(manualOffsets);// gives the calibration process a push so it is calibrated faster

    isCalibrated = false;// set calibration as not complete

    bno->setOprModeNdof();// operation mode used for faster calibration, sensor fusion and absolute orientation.
    
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

void ImuCallback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        msg_imu.is_calibration_complete = 0;// set calibration as not complete as this is continuously updated by the IMU

        cal = bno->getCalibration();// get calibration status for all sensors

        //check if calibration is done(i.e. if all sensor have calibration value 3)
        if(cal.gyro == 3 && cal.accel == 3 && cal.mag == 3){
            isCalibrated = true;
        }

        //if calibration is done send offsets
        if(isCalibrated){
            bno->setOprModeConfig(); // must be in config mode to get offset values
            vTaskDelay(100 / portTICK_PERIOD_MS);  // give IMU time to change operation mode 

            offsets = bno->getSensorOffsets(); // Get sensor offsets

            bno->setOprModeNdof();// change back the operation mode
            vTaskDelay(100 / portTICK_PERIOD_MS);  // give IMU time to change operation mode 

            // fill message with offsets
            msg_imu.offset_accel_x = offsets.accelOffsetX;
            msg_imu.offset_accel_y = offsets.accelOffsetY;
            msg_imu.offset_accel_z = offsets.accelOffsetZ;
            msg_imu.offset_accel_radius = offsets.accelRadius;
            msg_imu.offset_gyro_x = offsets.gyroOffsetX;
            msg_imu.offset_gyro_y = offsets.gyroOffsetY;
            msg_imu.offset_gyro_z = offsets.gyroOffsetZ;
            msg_imu.offset_mag_x = offsets.magOffsetX;
            msg_imu.offset_mag_y = offsets.magOffsetY;
            msg_imu.offset_mag_z = offsets.magOffsetZ;
            msg_imu.offset_mag_radius = offsets.magRadius;

            msg_imu.is_calibration_complete = 1;
        }
        
        //fill message with calibration status
        msg_imu.calibration_sys = cal.sys;
        msg_imu.calibration_gyro = cal.gyro;
        msg_imu.calibration_magnetometer = cal.mag;
        msg_imu.calibration_accelerometer = cal.accel;


        // fill message with orientation and acceleration
        q = bno->getQuaternion();         // [w, x, y, z]         quaternion container
        gravity = bno->getVectorGravity(); // [x, y, z]            gravity vector
        linAccel = bno->getVectorLinearAccel(); // [x, y, z]	    linear acceleration 
        GetHeading(ypr, &q, &gravity);
        msg_imu.yaw = ypr[0] * 180 / M_PI;
        msg_imu.pitch = ypr[1] * 180 / M_PI;
        msg_imu.roll = ypr[2] * 180 / M_PI;
        msg_imu.linear_acceleration_x = linAccel.x;
        msg_imu.linear_acceleration_y = linAccel.y;
        msg_imu.linear_acceleration_z = linAccel.z;



        // micro-ROS to publish to topic
        RCCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
        vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))

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
    RCCHECK(rclc_node_init_default(&node_imu, "imu_calibration_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher_imu, &node_imu, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, IMUCalibrationMessage), "/sensor/imu_calibration"));
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
