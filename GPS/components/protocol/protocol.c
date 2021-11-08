#include "protocol.h"

// configure adc sensor reading
void configure_adc(void) {
    // reset pins
    gpio_reset_pin(ADC1_CHANNEL_5);
    /* Set the GPIO as a push/pull output */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_6db);
}

// configure i2c master communication
void configure_i2c_master(void) {
    i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                         .sda_io_num = I2C_SDA_PORT,
                         .scl_io_num = I2C_SCL_PORT,
                         .sda_pullup_en = GPIO_PULLUP_ENABLE,
                         .scl_pullup_en = GPIO_PULLUP_ENABLE,
                         .master.clk_speed = I2C_MASTER_FREQ_HZ,
                         .clk_flags = 0};

    // configure and install the i2c driver
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_read(i2c_port_t i2c_num, uint8_t *data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }

    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}