/*
 *  UI.cpp
 *
 *  Created on: Jan 10, 2024
 * 
 *  Author: onlydcx, G4T1PR0
 */

#include "UI.hpp"
#include <McuAbstractionLayer/stm32f446AbstractionLayer.hpp>

const MAL::Peripheral_GPIO RotaryPin[4] = {
    MAL::Peripheral_GPIO::Rotary_IN0,
    MAL::Peripheral_GPIO::Rotary_IN1,
    MAL::Peripheral_GPIO::Rotary_IN2,
    MAL::Peripheral_GPIO::Rotary_IN3,
};

const MAL::Peripheral_GPIO LED[3] = {
    MAL::Peripheral_GPIO::Debug_LED0,
    MAL::Peripheral_GPIO::Debug_LED1,
    MAL::Peripheral_GPIO::Debug_LED2,
};

UI::UI(MAL* mcu) {
    _mcu = mcu;
}

void UI::buzzer(uint16_t pulse, uint8_t tim) {
    _mcu->pwmSetDuty(MAL::Peripheral_PWM::Buzzer, pulse / 4096);
}

uint8_t UI::getRotarySW() {
    bool _states[4] = {0};
    for(int i = 0; i < 4; i++) {
        _states[i] = _mcu->gpioGetValue(RotaryPin[i]);
    }
    return ((_states[0] >> 8) || (_states[1] >> 4) || (_states[2] >> 2) || (_states[3] >> 1));
}

void UI::setLED(uint8_t pin, bool states) {
    _mcu->gpioSetValue(LED[pin], states);
}