#include "string.h"
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//I2C
//Needed for the i2c config
#define I2C_MASTER_FREQ_HZ 100000 
#define I2C_MASTER_NUM 0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000
//Address to the GPS module
#define SLAVE_ADDR 0x42 

#ifndef HEADER_PROT
#define HEADER_PROT

/**
 * @brief Configures the adc for initializations and normal usage
 * 
 */

void configure_adc(void);

/**
 * @brief Configures i2c for master mode
 * 
 */
void configure_i2c_master(void);

#endif