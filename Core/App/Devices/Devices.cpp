/*
 * Devices.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <Devices/Devices.hpp>

Devices::Devices() {
    mcu = new stm32f446AbstractionLayer();
    lineSensor = new lineSensorDriver(mcu);
}

void Devices::init() {
    mcu->init();
}

void Devices::update() {
    lineSensor->update();
}
