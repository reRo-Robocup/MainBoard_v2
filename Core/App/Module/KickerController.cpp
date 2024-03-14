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
        case 0:  // 放電
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerH, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLA, true);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLB, true);
            break;

        case 1:  // チャージ
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLA, false);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLB, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerH, true);
            break;

        case 2:  // Aキック
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerH, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLA, true);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLB, false);
            _kick_start_time = _mcu->millis();
            _mode = 4;
            break;

        case 3:  // Bキック
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerH, false);

            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLA, true);
            _mcu->gpioSetValue(MAL::Peripheral_GPIO::KickerLB, false);
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