/*
 * HardwareController.cpp
 *
 *  Created on: Dec 8, 2023
 */

#include <HardwareController/HardwareController.hpp>

HardwareController::HardwareController(Devices* devices) {
    _devices = devices;
    lineSensorAlgo = new lineSensorAlgorithm(_devices);
    motor = new MotorController(_devices);
    camera = new CameraController(_devices);
}

void HardwareController::init() {
    lineSensorAlgo->init();
    motor->init();
    camera->init();
}

void HardwareController::update() {
    lineSensorAlgo->update();
}