/*
 *  MPU6500.cpp
 *
 *  Created on: Dec 23, 2023
 *
 *  Author: iguchi, SHIMOTORI Haruki, (onlydcx, G4T1PR0)
 *
 *  From : reRo_robotrace_board_public
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/MPU6500.c
 *  https://github.com/shimotoriharuki/reRo_robotrace_board_public/blob/master/main_code/Core/Src/IMU.cpp
 */

#include "MPU6500.hpp"

MPU6500::MPU6500(MAL* mcu) {
    _mcu = mcu;
}

void MPU6500::init() {
    uint8_t who_am_i;
    _dt = CONTROLL_CYCLE;
    who_am_i = _read_byte(0x75);
    if (who_am_i == 0x70) {
        _write_byte(0x6B, 0x00);  // sleep mode解除
        _mcu->delay_ms(100);
        _write_byte(0x1A, 0x00);
        //_write_byte(0x1B, 0x18);
        _write_byte(0x1B, 0x00);
        isInitialized = 1;
    }
    isCalibrationed = 0;
}

void MPU6500::calibration() {
    printf("IMU Calibration START\n\r");
    _offset_Zero();
    isCalibrationed = true;
    printf("imu Z offset : %d", _drift_constant);
    printf("IMU Calibration END\n\r");
}

void MPU6500::update() {
    if (isInitialized && isCalibrationed) {
        _read_gyro_data();
        _read_accel_data();

        Gx = (float)(rGx / 131.0);
        Gy = (float)(rGy / 131.0);
        Gz = (float)(rGz / 131.0);

        Ax = (float)(rAx / 16384.0);
        Ay = (float)(rAy / 16384.0);
        Az = (float)(rAz / 16384.0);

        Yaw += Gz * 0.001;
    }
}

void MPU6500::_read_gyro_data() {
    // rGx = ((int16_t)read_byte(0x43) << 8) | ((int16_t)read_byte(0x44));
    // rGy = ((int16_t)read_byte(0x45) << 8) | ((int16_t)read_byte(0x46));
    rGz = ((int16_t)_read_byte(0x47) << 8) | ((int16_t)_read_byte(0x48));
    rGz -= _drift_constant;
}

void MPU6500::_read_accel_data() {
    rAx = ((int16_t)_read_byte(0x3B) << 8) | ((int16_t)_read_byte(0x3C));
    rAy = ((int16_t)_read_byte(0x3D) << 8) | ((int16_t)_read_byte(0x3E));
    rAz = ((int16_t)_read_byte(0x3F) << 8) | ((int16_t)_read_byte(0x40));
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


void MPU6500::_offset_Zero() {
    uint32_t tim = _mcu->millis();
    uint16_t cnt = 40000;
    int32_t sum = 0;
    for (int i = 0; i < cnt; i++) {
    	this->_read_gyro_data();
    	sum += rGz;
    }
    _drift_constant = sum / cnt;
}
