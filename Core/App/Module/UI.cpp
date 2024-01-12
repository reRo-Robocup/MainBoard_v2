/*
 * UI.cpp
 *
 *  Created on: Jan 10, 2024
 */

#include "UI.hpp"

Buzzer::Buzzer(MAL* mcu) {
    _mcu = mcu;
}

void Buzzer::on(uint16_t pulse, uint8_t ms) {
    pulse /= _mcu->getPWMConpare(MAL::Peripheral_PWM::Buzzer);
    _mcu->pwmSetDuty(MAL::Peripheral_PWM::Buzzer, pulse);
}