/*
 * MPU6500.cpp
 *
 *  Created on: Dec 23, 2023
 */

#include <Devices/Driver/nMPU6500.hpp>

MPU6500::MPU6500(MAL* mcu) {
    _mcu = mcu;
}

void MPU6500::init() {
}

void MPU6500::update() {
}

uint8_t MPU6500::_read_byte(uint8_t reg) {
}
void MPU6500::_write_byte(uint8_t reg, uint8_t val) {
}
