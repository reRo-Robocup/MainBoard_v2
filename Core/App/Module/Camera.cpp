/*
 *  Camera.cpp
 *
 *  Created on: Jan 1, 2024
 * 
 *  Author: User, G4T1PR0
 */

#include <Camera.hpp>

#define YELLOW 0
#define BLUE 1

camera::camera(MAL* mcu) {
    _mcu = mcu;
}

void camera::init() {

}

void camera::updateFPS() {
    uint32_t _buffer = 0;
    _buffer = _mcu->uartGetChar(MAL::Peripheral_UART::Cam);
}

bool camera::getAttackColor() {
    return this->_AttackColor;
}