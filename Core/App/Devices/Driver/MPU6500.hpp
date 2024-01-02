/*
 *  nMPU6500.hpp
 *
 *  Created on: Dec 23, 2023
 * 
 *  Author: iguchi, SHIMOTORI Haruki
 * 
 *  From : reRo_robotrace_board_public
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/MPU6500.c
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/IMU.cpp
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

    int16_t yaw;
    float _dt;

    bool isInitialized;
    bool isCalibrationed;

    bool isRobotLift;

   private:
    MAL* _mcu;

    int16_t _drift_constant;

    void _read_gyro_data();
    void _read_accel_data();

    void _write_byte(uint8_t reg, uint8_t val);
    uint8_t _read_byte(uint8_t reg);
};

#endif /* APP_DEVICES_DRIVER_MPU6500_HPP_ */