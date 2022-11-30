#include "INA219.h"

#ifdef __cplusplus
extern "C" {
#endif
    typedef void * INA219Handle;
    INA219Handle create_INA219();
    void    free_INA219(INA219Handle p);

    void beginINA219(INA219Handle p, i2c_port_t v_i2c_num,gpio_num_t sda_io_num,gpio_num_t scl_io_num);
    float shuntVoltageINA219(INA219Handle p);

#ifdef __cplusplus
}
#endif