#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <autosail_message/msg/sail_angle_message.h>
//#include <autosail_message/msg/sail_furl_message.h>
#include <autosail_message/msg/rudder_control_message.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h> 

#include "components/INA219/INA219.cpp"
#include "components/INA219/include/INA219.h"

#ifdef ESP_PLATFORM
#include "driver/i2c.h"
#include "driver/timer.h"
#include "driver/dac.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

// You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000
// microseconds
#define SERVO_MIN_PULSEWIDTH_US 800     // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2200    // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90             // Maximum angle in degree upto which servo can rotate
#define SERVO_MIN_DEGREE 0            // Minimum angle
#define SERVO_PULSE_GPIO_SAIL GPIO_NUM_19        // GPIO connects to the PWM signal line
//#define SERVO_PULSE_GPIO_RUDDER GPIO_NUM_18      // GPIO connects to the PWM signal line
#define DACpin DAC_CHANNEL_1 // PIN 25

#define MS_DELAY 50  // Callback delay
#define SAIL_MOTOR_DELAY 40  // Callback delay
#define ZERO_FREQ 0 // Stop Frequency
#define DEFAULT_FREQ 1000 // Start Frequency
#define MAX_FREQ 6000 // Max Frequency
#define STEP_FREQ 200 // Step Frequency
#define DIR_PIN GPIO_NUM_18 // Direction pin

// C++ code
INA219 ina;
void InitBoomReading();
float getTrueSailAngle();

// C code
extern "C"{

rcl_subscription_t sub_sail;
rcl_subscription_t sub_rudder;
rcl_subscription_t sub_furl;
autosail_message__msg__SailAngleMessage angle_msg;
autosail_message__msg__RudderControlMessage rudder_msg;
//autosail_message__msg__SailFurlMessage furl_msg;

struct timeval startTime;
struct timeval currTime;
struct timeval prevTimeSail;
struct timeval prevTimeRudder;
struct timeval prevTimeFurl;

int32_t true_sail_angle = 0;
int32_t angle = 0;

ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK,
};

ledc_channel_config_t ledc_channel = {
    .gpio_num   = SERVO_PULSE_GPIO_SAIL,

    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel    = LEDC_CHANNEL_0,

    .timer_sel  = LEDC_TIMER_0,

    .duty       = 0, // Start out stopped (0% duty cycle)
    .hpoint     = 0,
};


int32_t sail_angle_set(int32_t sail_angle) {
    true_sail_angle = (int32_t)getTrueSailAngle();
    int32_t freq = DEFAULT_FREQ;
    int32_t sail_angle_diff = sail_angle - true_sail_angle;
    int32_t max_speed = abs(sail_angle_diff*STEP_FREQ);
    if (sail_angle > true_sail_angle) {
        gpio_set_level(DIR_PIN, 1);
    }
    else if (sail_angle < true_sail_angle) {
        gpio_set_level(DIR_PIN, 0);
    }
    while ((true_sail_angle < sail_angle - 2) || (true_sail_angle > sail_angle + 2)) {
        freq = freq + STEP_FREQ;
        if (freq > max_speed || freq > MAX_FREQ) {
            freq = max_speed;
            break;
        }
        ledc_set_freq(ledc_timer.speed_mode, ledc_timer.timer_num, abs(freq));
        vTaskDelay(pdMS_TO_TICKS(SAIL_MOTOR_DELAY));
    }
    // ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    // ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    return 1;
}


void sail_velocity_control(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 127);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    sail_angle_set(angle);
    vTaskDelay(pdMS_TO_TICKS(1000));
    angle = -angle;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 127);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    sail_angle_set(angle);
    vTaskDelay(pdMS_TO_TICKS(1000));
    angle = -angle;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sail_callback(const void *msgin) {
    const autosail_message__msg__SailAngleMessage *msg = (const autosail_message__msg__SailAngleMessage *)msgin;

    // Use angle to calculate PMW

    angle = msg->sail_angle;  // Data sent in degrees

    gettimeofday(&prevTimeSail, NULL);
}

void rudder_callback(const void *msgin) {
    const autosail_message__msg__RudderControlMessage *msg = (const autosail_message__msg__RudderControlMessage *)msgin;

    // Use angle to calculate output voltage
    int32_t rudder_angle = msg->rudder_angle;  // Data sent in degrees
    printf("Rudder angle received: %d\r\n", rudder_angle);
    
    if (rudder_angle > 45) {
        rudder_angle = 45.0;
    }
    else if (rudder_angle < -45){
        rudder_angle = -45.0;
    }
    
    uint8_t dac_value = ((rudder_angle^2) / 4050) + ((17 * rudder_angle)/6) + 127;

    dac_output_voltage(DACpin, dac_value);
    vTaskDelay(pdMS_TO_TICKS(MS_DELAY)); 

    gettimeofday(&prevTimeRudder, NULL);
}

/*
void furl_callback(const void *msgin) {
    gettimeofday(&prevTimeFurl, NULL);
}
*/

void appMain(void* arg) {

    while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1));

    InitBoomReading();

    gettimeofday(&startTime, NULL);
    prevTimeSail = startTime;
    prevTimeRudder = startTime;


    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

    
    RCCHECK(rclc_subscription_init_default(&sub_sail, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, SailAngleMessage),
                                           "/actuator/sail"));
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    RCCHECK(rclc_subscription_init_default(&sub_rudder, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, RudderControlMessage),
                                           "/actuator/rudder"));
    
    /*
    vTaskDelay(500 / portTICK_PERIOD_MS);
    RCCHECK(rclc_subscription_init_default(&sub_furl, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(autosail_message, msg, SailFurlMessage),
                                           "/actuator/furl"));
    */

    vTaskDelay(500 / portTICK_PERIOD_MS);
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, 0, sail_velocity_control));
    // xTaskCreate(sail_velocity_control, "Sail velocity control", configMINIMAL_STACK_SIZE, NULL, 1, &sail_velocity_handle);

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    rclc_executor_t executor_timer = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_timer, &support.context, 2, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor_timer, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_sail, &angle_msg, &sail_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_rudder, &rudder_msg, &rudder_callback, ON_NEW_DATA));
    // RCCHECK(rclc_executor_add_subscription(&executor, &sub_furl, &furl_msg, &furl_callback, ON_NEW_DATA));
    
    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_channel);
    
    // Initialize DAC
    dac_i2s_enable();
    dac_i2s_disable();
    dac_output_enable(DACpin);

    // Initialize GPIO for motor direction
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor_timer, RCL_MS_TO_NS(100));
        gettimeofday(&currTime, NULL);
        // if (((prevTimeSail.tv_sec > 1) && (prevTimeRudder.tv_sec > 1)) || RMW_RET_OK != rmw_uros_ping_agent(1000, 1))
            // if (((currTime.tv_sec - prevTimeSail.tv_sec) >= 3) || ((currTime.tv_sec - prevTimeRudder.tv_sec) >= 3)) {
                // settimeofday(&startTime, NULL);
                // esp_restart();
            // }
        // usleep(100);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&sub_sail, &node));
    RCCHECK(rcl_subscription_fini(&sub_rudder, &node));
    RCCHECK(rcl_subscription_fini(&sub_furl, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support));
    RCCHECK(rclc_executor_fini(&executor_timer));

    dac_output_disable(DACpin);
}

}

void InitBoomReading() {
    ina.begin(I2C_NUM_1,GPIO_NUM_21,GPIO_NUM_22);

    vTaskDelay(500 / portTICK_PERIOD_MS);
}

float getTrueSailAngle(){
    // measure the voltage from the angle sensor and convert to current(mA)
    float angle_current_mA = ina.shuntCurrent()*10000;//convert to current
    angle_current_mA = abs(angle_current_mA);

    // boom angle can simply be described with a linear formula. 4mA = 0/360deg. 8mA = 90deg 
    float measured_boom_angle = 22.5*angle_current_mA-90;
    
    // offset depending on how the sensor is positioned
    measured_boom_angle -= 90;

    // rewrite angle to be in the span of +-180 degrees
    if(measured_boom_angle > 180)
        measured_boom_angle-=360;

    return measured_boom_angle;
}