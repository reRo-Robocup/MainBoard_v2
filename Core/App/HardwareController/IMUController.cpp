/*
 * IMUController.cpp
 *
 *  Created on: Dec 23, 2023
 */

#include <math.h>
#include <Devices/Devices.hpp>
#include <HardwareController/IMUController.hpp>

IMUController::IMUController(Devices* devices) {
    _devices = devices;
};

void IMUController::init() {
    
}