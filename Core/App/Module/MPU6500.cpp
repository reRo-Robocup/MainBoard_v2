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
    _mode = 0;
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
    _Gz_drift_constant = 0;
}

void MPU6500::update() {
    switch (_mode) {
        case 0:  //
            if (isInitialized) {
                _mode = 1;
            }
            break;

        case 1:
            _calibration_start_time = _mcu->millis();
            _calibration_sum_cnt = 0;
            _read_gyro_data();
            if (raw_Gz != 0) {
                _mode = 2;
            }
            break;

        case 2:
            if (_mcu->millis() - _calibration_start_time > 1000) {
                _Gz_drift_constant = _calibration_Gz / _calibration_sum_cnt;
                _mode = 10;
            } else {
                _read_gyro_data();
                _calibration_Gz += raw_Gz;
                _calibration_sum_cnt++;
            }

            break;

        case 10:
            _read_gyro_data();
            _read_accel_data();

            Gx = (float)(raw_Gx / 131.0);
            Gy = (float)(raw_Gy / 131.0);
            Gz = (float)(raw_Gz / 131.0);

            Ax = (float)(raw_Ax / 16384.0);
            Ay = (float)(raw_Ay / 16384.0);
            Az = (float)(raw_Az / 16384.0);

            Yaw += Gz * 0.001;
            break;

        default:
            break;
    }
}

void MPU6500::_read_gyro_data() {
    // rGx = ((int16_t)read_byte(0x43) << 8) | ((int16_t)read_byte(0x44));
    // rGy = ((int16_t)read_byte(0x45) << 8) | ((int16_t)read_byte(0x46));
    raw_Gz = ((int16_t)_read_byte(0x47) << 8) | ((int16_t)_read_byte(0x48));
    raw_Gz -= _Gz_drift_constant;
}

void MPU6500::_read_accel_data() {
    raw_Ax = ((int16_t)_read_byte(0x3B) << 8) | ((int16_t)_read_byte(0x3C));
    raw_Ay = ((int16_t)_read_byte(0x3D) << 8) | ((int16_t)_read_byte(0x3E));
    raw_Az = ((int16_t)_read_byte(0x3F) << 8) | ((int16_t)_read_byte(0x40));
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
