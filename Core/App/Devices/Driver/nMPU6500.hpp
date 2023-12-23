/*
 * MPU6500.hpp
 *
 *  Created on: Dec 23, 2023
 */

#ifndef APP_DEVICES_DRIVER_MPU6500_HPP_
#define APP_DEVICES_DRIVER_MPU6500_HPP_
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class MPU6500 {
   public:
    MPU6500(MAL* mcu);
    void init();
    void update();

    bool isInitialized;

   private:
    MAL* _mcu;

    uint8_t _read_byte(uint8_t reg);
    void _write_byte(uint8_t reg, uint8_t val);

    int16_t xa, ya, za;
    int16_t xg, yg, zg;
};

#endif /* APP_DEVICES_DRIVER_MPU6500_HPP_ */