/*
 *  MPU6500.hpp
 *
 *  Created on: Dec 23, 2023
 *
 *  Author: iguchi, SHIMOTORI Haruki, (onlydcx, G4T1PR0)
 *
 *  From : reRo_robotrace_board_public
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/MPU6500.c
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/IMU.cpp
 */

#ifndef _APP_MODULE_MPU6500_HPP_
#define _APP_MODULE_MPU6500_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class MPU6500 {
   public:
    MPU6500(MAL* mcu);
    void init();
    void update();
    void calibration();

    int16_t rAx, rAy, rAz;  // raw value
    int16_t rGx, rGy, rGz;  // raw value

    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    float Yaw, Pitch, Roll;
    float Vy, Vx, Vz;

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

     void _offset_Zero();
};

#endif /* _APP_MODULE_MPU6500_HPP_ */