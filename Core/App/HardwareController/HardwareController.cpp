/*
 * HardwareController.cpp
 *
 *  Created on: Dec 8, 2023
 */

#include <HardwareController/HardwareController.hpp>

HardwareController::HardwareController(Devices* devices) {
    _devices = devices;
    lineSensorAlgo = new lineSensorAlgorithm(_devices);
}

void HardwareController::init() {
    lineSensorAlgo->init();
}

void HardwareController::update() {
    lineSensorAlgo->update();
}