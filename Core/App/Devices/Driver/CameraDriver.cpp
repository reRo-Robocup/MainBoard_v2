/*
 * CameraDriver.cpp
 *
 *  Created on: Dec 26, 2023
 */

#include <Devices/Driver/CameraDriver.hpp>

CameraDriver::CameraDriver(MAL* mcu) {
    _mcu = mcu;
}

void CameraDriver::init() {
    AttackColor = YELLOW;
}

void CameraDriver::updateFPS {
    
}