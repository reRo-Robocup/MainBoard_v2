/*
 *  BallControll.cpp
 *
 *  Created on: Dec 22, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#include "BallControll.hpp"

const MAL::Peripheral_ADC ballsensor[2] = {
    MAL::Peripheral_ADC::BallCatchA,
    MAL::Peripheral_ADC::BallCatchB
};

BallControll::BallControll(MAL* mcu) {
    _mcu = mcu;
}

void BallControll::init() {

}

void BallControll::setBallThreshold(uint8_t valA, uint8_t valB) {
    this->_threshold[0] = valA;
    this->_threshold[1] = valB;
}

bool BallControll::isCatch(uint8_t pin) {
    return (_mcu->adcGetValue(ballsensor[pin]) > _threshold[pin]);
}