/*
 * IMUController.hpp
 *
 *  Created on: Dec 23, 2023
 */

#include <math.h>
#include <Devices/Devices.hpp>
#include <HardwareController/CameraController.hpp>

#include "GlobalDefines.h"

int16_t angles[3] = {0};

CameraController::CameraController(Devices* devices) {
    _devices = devices;
}

void CameraController::init() {

}

void CameraController::update() {
    _devices->camera->updateFPS();
}