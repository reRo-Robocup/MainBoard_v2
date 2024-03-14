/*
 *  UI.cpp
 *
 *  Created on: Jan 10, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "UI.hpp"
#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>

const MAL::Peripheral_GPIO RotaryPin[4] = {
    MAL::Peripheral_GPIO::Rotary_IN1,
    MAL::Peripheral_GPIO::Rotary_IN0,
    MAL::Peripheral_GPIO::Rotary_IN3,
    MAL::Peripheral_GPIO::Rotary_IN2,
};

const MAL::Peripheral_GPIO LED[3] = {
    MAL::Peripheral_GPIO::Debug_LED0,
    MAL::Peripheral_GPIO::Debug_LED2,
    MAL::Peripheral_GPIO::Debug_LED1,
};

UI::UI(MAL* mcu) {
    _mcu = mcu;
}

void UI::init() {
    for (int i = 0; i < 3; i++) {
        this->setLED(i, 1);
    }
}

void UI::update() {
    switch (_buzzer_mode) {
        case 0:
            if (_buzzer_time > 0) {
                _mcu->pwmSetFrequency(MAL::Peripheral_PWM::Buzzer, _buzzer_Hz);
                _mcu->pwmSetDuty(MAL::Peripheral_PWM::Buzzer, 0.5);
                _buzzer_mode = 1;
            }
            break;

        case 1:
            if ((_mcu->millis() - _buzzer_start_time) > _buzzer_time) {
                _mcu->pwmSetDuty(MAL::Peripheral_PWM::Buzzer, 0);
                _buzzer_time = 0;
                _buzzer_mode = 0;
            }

        default:
            break;
    }
}

void UI::buzzer(uint32_t Hz, uint32_t tim) {
    _buzzer_start_time = _mcu->millis();
    _buzzer_time = tim;
    _buzzer_Hz = Hz;
}

uint8_t UI::getRotarySW() {
    bool bit[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        bit[i] = _mcu->gpioGetValue(RotaryPin[i]);
    }

    return bit[0] + bit[1] * 2 + bit[2] * 4 + bit[3] * 8;
}

bool UI::getSW() {
    return _mcu->gpioGetValue(MAL::Peripheral_GPIO::Debug_SW);
}

void UI::setLED(uint8_t pin, bool states) {
    _mcu->gpioSetValue(LED[pin], states);
}

void UI::Lchika() {
    for (int i = 0; i < 3; i++) {
        this->setLED(i, 0);
        _mcu->delay_ms(200);
    }
    for (int i = 0; i < 3; i++) {
        this->setLED(i, 1);
        _mcu->delay_ms(200);
    }
}