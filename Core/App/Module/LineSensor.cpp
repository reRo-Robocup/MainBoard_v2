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
    // this->_module_r = 11;
    for (int i = 0; i < 8; i++) {
        _mcu->gpioSetValue(MuxPin[i], 0);
    }
    for (int i = 0; i < 32; i++) {
        this->_sin_table[i] = sin((360 / 32 * i) * (M_PI / 180));
        this->_cos_table[i] = cos((360 / 32 * i) * (M_PI / 180));
        this->_threshold[i] = 700;
        // printf("TrigTable %u: Sin:%f Cos:%f\n", i, _SinCosTable[i][1], _SinCosTable[i][0]);
    }
    // setThreshold();
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

void LineSensor::update() {
    this->read();
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
        // printf("%u ", this->isSensorONline[i]);
    }
    // printf("\n");

    float _angle = 0;

    if (isonLine && (x != 0) && (y != 0)) {
        _angle = atan2(y, x);
        _angle *= (180 / M_PI);
        _angle = 90 - _angle;
        while(_angle < -180) _angle += 360;
        while(_angle >= 180) _angle -= 360;
    } else {
        _angle = 1023;
    }


    this->isonLine = (_cnt > 0);
    this->angle = (int16_t)_angle;
    this->_isONline_qty = _cnt;
}

int16_t LineSensor::getMoveAngle(int16_t yaw, int16_t toMove) {
    int16_t _angle = toMove;
    if (!this->isonLine) {
        // 反応してない時
        return _angle;
    } else {
        // 反応してる
        float _sens_Xvector, _sens_Yvector;
        _sens_Xvector = cos(this->angle * deg_to_rad);
        _sens_Yvector = sin(this->angle * deg_to_rad);
        float Xvector, Yvector;
        Xvector = cos(toMove * deg_to_rad) + _sens_Xvector;
        Yvector = sin(toMove * deg_to_rad) + _sens_Yvector;
        _angle = atan2(Yvector, Xvector) * rad_to_deg;
    }
    return _angle;
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

uint8_t LineSensor::getDisFromCenter() {
    // アルゴリズム
    // 反応してるセンサの個数を取得
    // そのセンサの座標を取得
    // 全ての座標同士の距離を求め、最大値とその時の2つの座標を取る
    // 傾きから法線を求め、(0,0)からの距離を求める

    if (this->_isONline_qty > 1) {
        // 反応したセンサの座標だけが入る配列
        const uint8_t num = _isONline_qty;
        float _isONline_sensorXY[num][2];
        uint8_t _cnt = 0;
        for (int i = 0; i < 32; i++) {
            if (this->sensor_isonline[i]) {
                _isONline_sensorXY[i][0] = _cos_table[i];
                _isONline_sensorXY[i][1] = _sin_table[i];
                _cnt++;
            }
        }

        // 総当たりでセンサ同士の距離を調べる
        float maxdis;
        for (int i = 0; i < num; i++) {
            for (int j = num; j >= 0; j--) {
                if (i != j) {
                    float _dx, _dy, _dis;
                    // _dx = abs(_sens_XYvector[i][0] - _sens_XYvector[j][0]);
                    // _dy = abs(_sens_XYvector[i][1] - _sens_XYvector[j][1]);
                    _dis = sqrt(pow(_dx, 2) + pow(_dy, 2));
                    if (maxdis < _dis) {
                        maxdis = _dis;
                    }
                }
            }
        }
        // map2_180(slope);
    }
}
