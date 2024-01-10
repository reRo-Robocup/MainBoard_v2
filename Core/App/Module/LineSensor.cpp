/*
 * LineSensor.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <LineSensor.hpp>

LineSensor::LineSensor(MAL* mcu) {
    _mcu = mcu;
}

void LineSensor::init() {
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig0, 0);
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig1, 0);
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig2, 0);
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig3, 0);
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig0, 0);
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig1, 0);
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig2, 0);
    _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig3, 0);
}

void LineSensor::update() {
    for (int i = 0; i < 16; i++) {
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig0, SigPattern[i][0]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig1, SigPattern[i][1]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig2, SigPattern[i][2]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig3, SigPattern[i][3]);

        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig0, SigPattern[i][0]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig1, SigPattern[i][1]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig2, SigPattern[i][2]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig3, SigPattern[i][3]);

        //_mcu->wait_ms();
        while(!_mcu->isAdcConvCplt(MAL::Peripheral_ADC::MuxA))
            ;
        while(!_mcu->isAdcConvCplt(MAL::Peripheral_ADC::MuxB))
            ;

        sensorValue[i]      = _mcu->adcGetValue(MAL::Peripheral_ADC::MuxA);
        sensorValue[i + 16] = _mcu->adcGetValue(MAL::Peripheral_ADC::MuxB);
    }
}

void LineSensor::setThreshold() {
    uint16_t _up_tim = 3000;
    uint32_t _tim = _mcu->millis();
    uint16_t _minVal[32] = {4096}, _maxVal[32] = {0};
    while((_mcu->millis() - _tim) < _up_tim) {
        LineSensor::update();
        for(int i = 0; i < 16; i++) {
            if(_minVal[i] > this->sensorValue[i])    
                _minVal[i] = this->sensorValue[i];
            if(_minVal[i+16] > this->sensorValue[i+16]) 
                _minVal[i+16] = this->sensorValue[i+16];
            if(_maxVal[i] < this->sensorValue[i])    
                _minVal[i] = this->sensorValue[i];
            if(_maxVal[i+16] < this->sensorValue[i+32]) 
                _minVal[i+32] = this->sensorValue[i+32];
        }
    }
    for(int i = 0; i < 16; i++) {
        this->_threshold[i] = _minVal[i] + (_maxVal[i] - _minVal[i]) / 2;
        this->_threshold[i+32] = _minVal[i] + (_maxVal[i] - _minVal[i]) / 2;
    }
}