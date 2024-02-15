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
        _write_byte(0x1B, 0x18);  // FS_SEL = 3
        //_write_byte(0x1B, 0x00); // FS_SEL = 0
        isInitialized = 1;
    }
    isCalibrationed = 0;

    _Gx_drift_constant = 0;
    _Gy_drift_constant = 0;
    _Gz_drift_constant = 0;
    Yaw = 180;
    _madgwick.begin(1000.0);
}

void MPU6500::update() {
    switch (_mode) {
        case 0:  //
            if (isInitialized) {
                _mode = 1;
            }
            break;

        case 1:
            _calibration_sum_cnt = 0;
            _read_gyro_data();
            if (raw_Gz != 0) {
                _mode = 2;
            }
            break;

        case 2:
            if (_calibration_sum_cnt >= 1000) {
                _Gx_drift_constant = _calibration_sum_Gx / _calibration_sum_cnt;
                _Gy_drift_constant = _calibration_sum_Gy / _calibration_sum_cnt;
                _Gz_drift_constant = _calibration_sum_Gz / _calibration_sum_cnt;

                isCalibrationed = true;
                _mode = 10;
            } else {
                _read_accel_data();
                _read_gyro_data();

                _calibration_sum_Gx += raw_Gx;
                _calibration_sum_Gy += raw_Gy;
                _calibration_sum_Gz += raw_Gz;
                _calibration_sum_cnt++;
            }

            break;

        case 10:
            _read_gyro_data();
            _read_accel_data();

            // Gx = (float)(raw_Gx / 131.0);
            // Gy = (float)(raw_Gy / 131.0);
            // Gz = (float)(raw_Gz / 131.0);

            Gx = (float)(raw_Gx / 16.4);
            Gy = (float)(raw_Gy / 16.4);
            Gz = (float)(raw_Gz / 16.4);

            Ax = (float)(raw_Ax / 16384.0);
            Ay = (float)(raw_Ay / 16384.0);
            Az = (float)(raw_Az / 16384.0);

            _madgwick.updateIMU(Gx, Gy, Gz, Ax, Ay, Az);

            // Yaw += Gz * 0.001;
            Yaw = _madgwick.getYaw();
            break;

        default:
            break;
    }
}

void MPU6500::_read_gyro_data() {
    raw_Gx = ((int16_t)_read_byte(0x43) << 8) | ((int16_t)_read_byte(0x44));
    raw_Gy = ((int16_t)_read_byte(0x45) << 8) | ((int16_t)_read_byte(0x46));
    raw_Gz = ((int16_t)_read_byte(0x47) << 8) | ((int16_t)_read_byte(0x48));

    raw_Gx -= _Gx_drift_constant;
    raw_Gy -= _Gy_drift_constant;
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
