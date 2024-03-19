/*
 *  LineSensor.cpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "LineSensor.hpp"

LineSensor::LineSensor(MAL* mcu, UI* ui) {
    _mcu = mcu;
    _ui = ui;
}

const MAL::Peripheral_GPIO MuxPin[8] = {
    MAL::Peripheral_GPIO::MuxA_Sig0,
    MAL::Peripheral_GPIO::MuxA_Sig1,
    MAL::Peripheral_GPIO::MuxA_Sig2,
    MAL::Peripheral_GPIO::MuxA_Sig3,
    MAL::Peripheral_GPIO::MuxB_Sig0,
    MAL::Peripheral_GPIO::MuxB_Sig1,
    MAL::Peripheral_GPIO::MuxB_Sig2,
    MAL::Peripheral_GPIO::MuxB_Sig3,
};

void LineSensor::init() {
    this->_module_r = 60;
    for (int i = 0; i < 8; i++) {
        _mcu->gpioSetValue(MuxPin[i], 0);
    }
    // printf("sensor zahyo\n");
    for (int i = 0; i < 32; i++) {
        this->_sin_table[i] = sin((360 / 32 * i) * (M_PI / 180));
        this->_cos_table[i] = cos((360 / 32 * i) * (M_PI / 180));
        this->_threshold[i] = 1000;

        this->_sens_x[i] = _module_r * _cos_table[i];
        this->_sens_y[i] = _module_r * _sin_table[i];

        // printf("i:%d %f\n",i, _sens_x[i]);
    }
}

void LineSensor::read() {
    for (int i = 0; i < 16; i++) {
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig0, SigPattern[i][0]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig1, SigPattern[i][1]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig2, SigPattern[i][2]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig3, SigPattern[i][3]);

        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig0, SigPattern[i][0]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig1, SigPattern[i][1]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig2, SigPattern[i][2]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig3, SigPattern[i][3]);

        _mcu->adcConvCpltClearFlag(MAL::Peripheral_ADC::MuxA);
        _mcu->adcConvCpltClearFlag(MAL::Peripheral_ADC::MuxB);

        while (!_mcu->adcConvCpltGetFlag(MAL::Peripheral_ADC::MuxA)) {
        }
        while (!_mcu->adcConvCpltGetFlag(MAL::Peripheral_ADC::MuxB)) {
        }

        sensorValue[i] = _mcu->adcGetValue(MAL::Peripheral_ADC::MuxA);
        sensorValue[i + 16] = _mcu->adcGetValue(MAL::Peripheral_ADC::MuxB);
    }
}

void LineSensor::_set() {
    const bool _states[32] = {
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };
    for(int i = 0; i < 32; i++) {
        if(_states[i]) sensorValue[i] = 1023;
        else sensorValue[i] = 0;
    }
}

void LineSensor::update() {
    this->read();
    // this->_set();
    double x, y;
    uint8_t _cnt = 0;

    for (int i = 0; i < 32; i++) {
        if (sensorValue[i] > _threshold[i]) {
            this->sensor_isonline[i] = 1;
            _cnt++;
            x += _cos_table[i];
            y += _sin_table[i];
        } else {
            this->sensor_isonline[i] = 0;
        }
    }
    float _angle = 0;

    if (isonLine && (x != 0) && (y != 0)) {
        _angle = atan2(y, x) * rad_to_deg;
        _angle = 270 - _angle;
        if(_angle < -180) _angle += 360;
        if(_angle >= 180) _angle -= 360;
    } else {
        _angle = 1023;
    }
    this->isonLine = (_cnt > 0);
    this->angle = (int16_t)_angle;
    this->isOn_qty = _cnt;
}

void LineSensor::setThreshold() {
    _ui->buzzer(1000, 100);
    uint16_t _up_tim = 3000;
    uint32_t _tim = _mcu->millis();
    uint16_t _minVal[32] = {4096}, _maxVal[32] = {0};
    while ((_mcu->millis() - _tim) < _up_tim) {
        this->update();
        for (int i = 0; i < 16; i++) {
            if (_minVal[i] > this->sensorValue[i])
                _minVal[i] = this->sensorValue[i];
            if (_minVal[i + 16] > this->sensorValue[i + 16])
                _minVal[i + 16] = this->sensorValue[i + 16];
            if (_maxVal[i] < this->sensorValue[i])
                _minVal[i] = this->sensorValue[i];
            if (_maxVal[i + 16] < this->sensorValue[i + 16])
                _minVal[i + 16] = this->sensorValue[i + 16];
        }
    }
    for (int i = 0; i < 16; i++) {
        this->_threshold[i] = _minVal[i] + (_maxVal[i] - _minVal[i]) / 2;
        this->_threshold[i + 16] = _minVal[i] + (_maxVal[i] - _minVal[i]) / 2;
    }
    _ui->buzzer(1000, 100);
}

float LineSensor::getSensDistance() {
    float val = 0;
    if((this->isOn_qty < 2) && (!this->isonLine)) {
        return val;
    }
    else {
        const uint8_t num = this->isOn_qty;

        float x_array[num] = {0};
        float y_array[num] = {0};
        uint8_t id_array[num] = {0};

        uint8_t _cnt = 0;

        for(int i = 0; i < 32; i++) {
            if(this->sensor_isonline[i]) {
                id_array[_cnt] = i;
                _cnt++;
            }
        }

        for(int i = 0; i < num; i++) {
            x_array[i] = this->_sens_x[id_array[i]];
            y_array[i] = this->_sens_y[id_array[i]];
        }

        float maxDistance = 0;

        for(int i = 0; i < num; i++) {
            for(int j = (num - 1); j >= 0; j--) {
                if(i != j) {
                    // float diff_x = abs(x_array[i]) - abs(x_array[j]);
                    // float diff_y = abs(y_array[i]) - abs(y_array[j]);
                    float diff_x = x_array[i] - x_array[j];
                    float diff_y = y_array[i] - y_array[j];
                    float dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
                    if(maxDistance < dis) maxDistance = dis;
                }
            }
        }
        val = maxDistance;
        return val;
    }
}