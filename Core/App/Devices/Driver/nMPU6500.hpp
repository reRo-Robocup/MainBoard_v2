/*
 * nMPU6500.hpp
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
    void calibration();

    int16_t xa, ya, za;
    int16_t xg, yg, zg;

    uint8_t yaw;
    float yaw_f;

    bool isInitialized;
    bool isCalibrationed;

   private:
    MAL* _mcu;

    float _dt;
    int16_t _drift_constant;

    void _read_gyro_data();
    void _read_accel_data();

    void _write_byte(uint8_t reg, uint8_t val);
    uint8_t _read_byte(uint8_t reg);
};

#endif /* APP_DEVICES_DRIVER_MPU6500_HPP_ */