#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

extern "C" {
	void app_main(void);
}

extern void task_init(void*);
extern void micro_ros_task(void * arg);

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
	
	while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1));
	
	xTaskCreate(&task_init, "mpu_task", 2048, NULL, 5, NULL);
    	//vTaskDelay(500/portTICK_PERIOD_MS);
	xTaskCreate(&micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
