/*
 *  nMPU6500.cpp
 *
 *  Created on: Dec 23, 2023
 * 
 *  Author: iguchi, SHIMOTORI Haruki
 * 
 *  From : reRo_robotrace_board_public
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/MPU6500.c
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/IMU.cpp
 */

#include <Devices/Driver/MPU6500.hpp>
#include "GlobalDefines.h"

#define ZA_LIFTED 9.8

MPU6500::MPU6500(MAL* mcu) {
    _mcu = mcu;
}

void MPU6500::init() {
    uint8_t who_am_i;
    _dt = IMU_CONTROLL_CYCLE;
    who_am_i = _read_byte(0x75);
    if (who_am_i == 0x70) {
        _write_byte(0x6B, 0x00);  // sleep mode解除
        _mcu->delay_ms(100);
        _write_byte(0x1A, 0x00);
        _write_byte(0x1B, 0x18);
        isInitialized = 1;
    }
    isCalibrationed = 0;
}

void MPU6500::calibration() {
    if(!isCalibrationed) {
        int16_t _tmp_za_min = INT16_MAX, _tmp_za_max = 0;
        int16_t _data = 0;
        int16_t constant = 0;
        uint32_t tim = _mcu->millis();
        while((_mcu->millis() - tim) < 2000) {
            MPU6500::_read_accel_data();
            _data = za;
            if(_data > _tmp_za_max) _data = _tmp_za_max;
            if(_data < _tmp_za_min) _data = _tmp_za_min;
        }
        _drift_constant = (_tmp_za_max - _tmp_za_min) + constant;
        isCalibrationed = 1;
    }
}

void MPU6500::update() {
    if (isInitialized && isCalibrationed) {
        _read_gyro_data();
        _read_accel_data();
    }
    yaw += _dt * za + _drift_constant;
    while(yaw > 180)  yaw -= 360;
    while(yaw < -180) yaw += 360;

    isRobotLift = (za > ZA_LIFTED);
}

void MPU6500::_read_gyro_data() {
    // xg = ((int16_t)read_byte(0x43) << 8) | ((int16_t)read_byte(0x44));
    // yg = ((int16_t)read_byte(0x45) << 8) | ((int16_t)read_byte(0x46));
    zg = ((int16_t)_read_byte(0x47) << 8) | ((int16_t)_read_byte(0x48));
}

void MPU6500::_read_accel_data() {
    // xa = ((int16_t)_read_byte(0x3B) << 8) | ((int16_t)_read_byte(0x3C));
    // ya = ((int16_t)_read_byte(0x3D) << 8) | ((int16_t)_read_byte(0x3E));
    za = ((int16_t)_read_byte(0x3F) << 8) | ((int16_t)_read_byte(0x40));
}

void MPU6500::_write_byte(uint8_t reg, uint8_t val) {
    uint8_t ret;
    ret = reg & 0x7F;

    _mcu->gpioSetValue(MAL::Peripheral_GPIO::IMU_CS, 0);

    _mcu->spiWriteViaBuffer(MAL::Peripheral_SPI::IMU, &ret, 1);
    _mcu->spiReadViaBuffer(MAL::Peripheral_SPI::IMU, &val, 1);

    _mcu->gpioSetValue(MAL::Peripheral_GPIO::IMU_CS, 1);
}

uint8_t MPU6500::_read_byte(uint8_t reg) {
    uint8_t ret, val;
    ret = reg | 0x80;

    _mcu->gpioSetValue(MAL::Peripheral_GPIO::IMU_CS, 0);

    _mcu->spiWriteViaBuffer(MAL::Peripheral_SPI::IMU, &ret, 1);
    _mcu->spiReadViaBuffer(MAL::Peripheral_SPI::IMU, &val, 1);

    _mcu->gpioSetValue(MAL::Peripheral_GPIO::IMU_CS, 1);

    return val;
}
