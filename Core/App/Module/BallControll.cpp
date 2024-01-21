/*
 *  BallControll.cpp
 *
 *  Created on: Dec 22, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#include "BallControll.hpp"

BallControll::BallControll(MAL* mcu) {
    _mcu = mcu;
}

void BallControll::init() {
    
}

void BallControll::setBallThreshold(uint8_t valA, uint8_t valB) {

}

bool BallControll::isCatch(MAL::Peripheral_ADC p) {
    return _mcu->adcGetValue(p);
}