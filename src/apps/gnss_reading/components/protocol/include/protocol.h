#include <stdio.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

// I2C
// Needed for the i2c config
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM 0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_SDA_PORT 21
#define I2C_SCL_PORT 22
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_VAL 0x0
#define NACK_VAL 0x1
// Address to the GPS module
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

esp_err_t i2c_read(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);

#endif
