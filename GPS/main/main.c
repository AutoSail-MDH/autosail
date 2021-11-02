#include "nmea.h"
#include "protocol.h"
#include "main.h"

int app_main(void)
{
    /*while(1){
        data = adc1_get_raw(ADC1_CHANNEL_5);
        printf("Sensor reading: %d\n",data);
        vTaskDelay(100);
    }*/
    int i = 0;
    int c = 0;
    float lon = 0;
    float lat = 0;
    uint8_t reg_addr = 0xFF; //output register
    uint8_t * data = calloc(100,4);
    char * msg = calloc(100,1);
    if(msg == NULL){
        printf("Calloc failed\n");
    }

    //i2c_driver_delete(I2C_MASTER_NUM);
    configure_i2c_master();

    while(1){
        //read data from the sensor
        i2c_master_write_read_device(I2C_MASTER_NUM, SLAVE_ADDR, &reg_addr, 1, data, 400, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);

        i = 0;
        lon = 0;
        lat = 0;
        while((data[i] != 42) && (i < 100)){
            msg[i] = (char)data[i];
            i++;
        }

        if(!getPos(msg, &lat, &lon)){
            c++;
            printf("No data avalible for %d samples\n", c);          
        }
        else{
            c = 0;
            printf("Lat: %.4f : Long: %.4f\n", lat, lon);
        }
        
        vTaskDelay(50);
    }
    return 0;
}
