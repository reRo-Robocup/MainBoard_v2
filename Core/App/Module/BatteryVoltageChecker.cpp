/*
 *  BatteryVoltageCheckcer.cpp
 *
 *  Created on: Mar 3, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "BatteryVoltageChecker.hpp"

BatteryVoltageChecker::BatteryVoltageChecker(baseMcuAbstractionLayer* mcu) {
    _mcu = mcu;
}

void BatteryVoltageChecker::init() {
    _filter.init();
    setWarningVoltage(10);
    setCriticalVoltage(8);
    setWarningTime(1000);
    setCriticalTime(1000);
    setWarningCallback(nullptr);
    setCriticalCallback(nullptr);

    _mode = 0;

    for (int i = 0; i < 50; i++) {
        _filter.push(_mcu->adcGetValue(MAL::Peripheral_ADC::BatteryVoltage));
    }
    _voltage = _filter.get() * _raw2voltage * _voltage2batt;
}

void BatteryVoltageChecker::update() {
    _filter.push(_mcu->adcGetValue(MAL::Peripheral_ADC::BatteryVoltage));
    _voltage = _filter.get() * _raw2voltage * _voltage2batt;

    switch (_mode) {
        case 0:
            if (_voltage < _warning_voltage) {
                _mode = 1;
                _warning_start_time = _mcu->millis();
            } else if (_voltage < _critical_voltage) {
                _mode = 10;
                _critical_start_time = _mcu->millis();
            } else {
                _mode = 0;
            }
            break;

        case 1:
            if (_voltage < _warning_voltage) {
                if (_voltage < _critical_voltage) {
                    _mode = 10;
                    _critical_start_time = _mcu->millis();
                }
                if (_mcu->millis() - _warning_start_time > _warning_time) {
                    _mode = 2;
                    if (_warning_callback != nullptr) {
                        _warning_callback(true);
                    }
                }
            } else {
                _mode = 0;
            }
            break;

        case 2:
            if (_voltage < _warning_voltage) {
                if (_voltage < _critical_voltage) {
                    _mode = 10;
                    _critical_start_time = _mcu->millis();
                }
                _mode = 2;
            } else {
                _warning_callback(false);
                _mode = 0;
            }
            break;

        case 10:
            if (_voltage > _critical_voltage) {
                if (_mcu->millis() - _critical_start_time > _critical_time) {
                    _mode = 11;
                    if (_critical_callback != nullptr) {
                        _critical_callback(true);
                    }
                }
            } else {
                _mode = 0;
            }
            break;

        case 11:
            if (_voltage > _critical_voltage) {
                _mode = 11;
            } else {
                _critical_callback(false);
                _mode = 0;
            }
            break;

        default:
            break;
    }
}

float BatteryVoltageChecker::getVoltage() {
    return _voltage;
}

void BatteryVoltageChecker::setWarningVoltage(float voltage) {
    _warning_voltage = voltage;
}

void BatteryVoltageChecker::setCriticalVoltage(float voltage) {
    _critical_voltage = voltage;
}

void BatteryVoltageChecker::setWarningTime(uint32_t time) {
    _warning_time = time;
}

void BatteryVoltageChecker::setCriticalTime(uint32_t time) {
    _critical_time = time;
}

void BatteryVoltageChecker::setWarningCallback(void (*callback)(bool)) {
    _warning_callback = callback;
}

void BatteryVoltageChecker::setCriticalCallback(void (*callback)(bool)) {
    _critical_callback = callback;
}