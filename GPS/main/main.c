#include "main.h"

#include "math.h"
#include "nmea.h"
#include "protocol.h"

int app_main(void) {
    // variables
    int i = 0;
    int c = 0;

    float lon = 0;
    float lat = 0;

    int iter = 0;

    // uint8_t reg_addr = 0xFF;  // output register

    uint8_t* data = calloc(100, 4);
    char* msg = calloc(100, 1);

    if (msg == NULL) {
        printf("Calloc for msg failed\n");
    }
    if (data == NULL) {
        printf("Calloc for data failed\n");
    }

    // configure i2c
    configure_i2c_master();

    while (1) {
        // read data from the sensor
        // i2c_master_write_read_device(I2C_MASTER_NUM, SLAVE_ADDR, &reg_addr, 1, data, 400,
        //                             I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
        i2c_read(I2C_MASTER_NUM, data, 400);

        // convert the data to char
        i = 0;
        lon = 0;
        lat = 0;
        // Only convert up to the * sign, since that marks the end of a message
        while ((data[i] != 42) && (i < 82)) {
            msg[i] = (char)data[i];
            i++;
        }

        // get the GPS position
        if (!getPos(msg, &lat, &lon)) {
            c++;
            // printf("No data avalible for %d readings\n", c);
        } else {
            c = 0;
            if ((int)floor(lat) != 59 || (int)floor(lon) != 16) {
                printf("%d: ", iter);
                printf("%s\n", msg);
                printf("\n");
            }
            // printf("Lat: %.4f : Long: %.4f\n", lat, lon);
        }

        // delay for easier to read prints
        // vTaskDelay(50);
        iter++;
    }
    return 0;
}
