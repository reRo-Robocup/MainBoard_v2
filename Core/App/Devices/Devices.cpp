/*
 * Devices.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <Devices/Devices.hpp>

Devices::Devices() {
    mcu = new stm32f446AbstractionLayer();
    lineSensor = new lineSensorDriver(mcu);
    mpu6500 = new MPU6500(mcu);
}

void Devices::init() {
    mcu->init();
    mpu6500->init();
}

void Devices::update() {
}
