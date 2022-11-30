#include "include/INA219interfaceC.h"
#include "include/INA219.h"

extern "C"
{
    INA219Handle create_INA219(){
      return (INA219*) new INA219();
    };
    void free_INA219(INA219Handle p){
      delete (INA219*) p;
    }

    void beginINA219(INA219Handle p, i2c_port_t v_i2c_num,gpio_num_t sda_io_num,gpio_num_t scl_io_num){
      return ((INA219*)p)->begin( v_i2c_num, sda_io_num, scl_io_num);
    }
    float shuntVoltageINA219(INA219Handle p){
      return ((INA219*)p)->shuntVoltage();
    }
}