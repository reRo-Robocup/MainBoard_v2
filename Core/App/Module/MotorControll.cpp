/*
 * MotorControll.cpp
 *
 *  Created on: Dec 23, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "MotorControll.hpp"

const bool isMotorPinReversed[4] = {
    0, 0, 1, 0};

const MAL::Peripheral_PWM motor[4] = {
    MAL::Peripheral_PWM::Motor1,
    MAL::Peripheral_PWM::Motor2,
    MAL::Peripheral_PWM::Motor3,
    MAL::Peripheral_PWM::Motor4,
};

const uint16_t _motorAngles[4] = {45, 135, 225, 315};

MotorControll::MotorControll(MAL* mcu, MPU6500* imu) {
    _imu = imu;
    _mcu = mcu;
}

void MotorControll::_write_pwm(uint8_t pin, float duty) {
    duty *= speed;
    if (isMotorPinReversed[pin])
        duty *= -1;
    duty = (duty + 1) / 2;
    _mcu->pwmSetDuty(motor[pin], duty);
}

void MotorControll::init() {
    for (int i = 0; i < 4; i++) {
        this->_write_pwm(motor[i], 0);
    }
    _turn_angle_pid.setPID(0.005, 0.0, 0);
    speed = 0.2;
}

float MotorControll::_getBatteryVoltage() {
    return _mcu->adcGetValue(MAL::Peripheral_ADC::BatteryVoltage);
}

bool MotorControll::isDRVsleep() {
    return _mcu->gpioGetValue(MAL::Peripheral_GPIO::isMotorEnabled);
}

void MotorControll::run(int16_t angle) {
    angle = 450 - angle;
    while (angle >= 360)
        angle -= 360;
    while (angle <= -360)
        angle += 360;
    float MPowerVector[4] = {0};  // 4つのモーターの出力比
    float MPowerMax = 0;          // 最大値

    for (int i = 0; i < 4; i++) {
        float tmp_angle = (angle - _motorAngles[i]) * (M_PI / 180);
        MPowerVector[i] = cos(tmp_angle);
        // printf("%f ", MPowerVector[i]);

        if (MPowerMax < MPowerVector[i])
            MPowerMax = MPowerVector[i];
    }
    // printf("\n");

    if ((MPowerMax != 1) || (MPowerMax != -1)) {
        for (int i = 0; i < 4; i++) {
            MPowerVector[i] *= (1 / MPowerMax);
        }
    }

    // if(abs(MPowerMax) > this->speed) {
    //     for(int )
    // }

    for (int i = 0; i < 4; i++) {
        this->_write_pwm(motor[i], MPowerVector[i] * speed);
    }
}

void MotorControll::turn(int16_t targetAngle, int16_t yaw) {
    int16_t diff = targetAngle - yaw;
    float duty[4] = {0};
    for (int i = 0; i < 4; i++) {
        this->roll(motor[i], duty[i]);
    }
}

void MotorControll::roll(uint8_t ch, float duty) {
    this->_write_pwm(motor[ch], duty);
}