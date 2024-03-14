/*
 *  KickerController.cpp
 *
 *  Created on: Mar 13, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "KickerController.hpp"

KickerController::KickerController(baseMcuAbstractionLayer* mcu) {
    _mcu = mcu;
}

void KickerController::init() {
    _mode = 0;
}

void KickerController::update() {
    switch (_mode) {
        case 0:
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_H, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LA, false);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LB, false);
            break;

        case 1:  // チャージ
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LA, false);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LB, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_H, true);
            break;

        case 2:  // Aキック
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_H, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LA, true);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LB, false);
            _kick_start_time = _mcu->millis();
            _mode = 4;
            break;

        case 3:  // Bキック
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_H, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LA, false);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::Kicker_LB, true);
            _kick_start_time = _mcu->millis();
            _mode = 4;
            break;

        case 4:
            if (_mcu->millis() - _kick_start_time > _kick_time) {
                _mode = 1;
            }
            break;

        default:
            _mode = 0;
            break;
    }
}

void KickerController::setMode(int mode) {
    _mode = mode;
}