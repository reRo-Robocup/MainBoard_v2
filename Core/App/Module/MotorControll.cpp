/*
 * MotorControll.cpp
 *
 *  Created on: Dec 23, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#include "MotorControll.hpp"
#include "GlobalDefines.h"

#define MOTOR_STOP_COMPARE (__HAL_TIM_GET_AUTORELOAD(&htim1) / 2)

const bool isMotorPinReversed[4] = {
    false, false, false, false
};

const MAL::Peripheral_PWM motor[4] = {
    MAL::Peripheral_PWM::Motor1,
    MAL::Peripheral_PWM::Motor2,
    MAL::Peripheral_PWM::Motor3,
    MAL::Peripheral_PWM::Motor4,
};

const float speed_constant = 0.2;
const uint16_t _motorAngles[4] = {45, 135, 225, 315};  // モーターの配置角度

MotorControll::MotorControll(MAL* mcu) {
    _mcu = mcu;
    speed = 100;
}

void MotorControll::init() {
    for(int i = 0; i < 4; i++) {
        _mcu->pwmSetDuty(motor[i], 0.5);
    }
}

float MotorControll::_getBatteryVoltage() {
    return _mcu->adcGetValue(MAL::Peripheral_ADC::BatteryVoltage) / 4096;
}

float MotorControll::_duty_to_LAPduty(float duty) {
    return (duty++)/2;
}

void MotorControll::run(uint8_t angle) {
    angle = 450 - angle;
    while (angle > 359)
        angle -= 360;
    while (angle < 0)
        angle += 360;
    float MPowerVector[4] = {0};  // 4つのモーターの出力比
    float MPowerMax = 0;          // 最大値
    for (int i = 0; i < 4; i++) {
        MPowerVector[i] = sin(deg_to_rad(angle - _motorAngles[i]));
        if (MPowerMax < MPowerVector[i])
            MPowerMax = MPowerVector[i];
        if (isMotorPinReversed)
            MPowerVector[i] *= -1;
    }
    if ((MPowerMax = !1) || (MPowerMax = !-1)) {
        for (int i = 0; i < 4; i++) {
            MPowerVector[i] *= (1 / MPowerMax);
        }
    }
    float _write_compare[4] = {0};
    float speed_constant = 0.5;
    for (int i = 0; i < 4; i++) {
        _write_compare[i] = MotorControll::_duty_to_LAPduty(MPowerVector[i]) * (this->speed / 100);
    }

    for(int i = 0; i < 4; i++) {
        _mcu->pwmSetDuty(motor[i], _write_compare[i]);
    }
}

void MotorControll::turn(bool cw) {
    for(int i = 0; i < 4; i++) {
        float _s = MotorControll::_duty_to_LAPduty(cw * (1 / speed));
        _mcu->pwmSetDuty(motor[i], _s);
    }
}

void MotorControll::carryBall(int16_t TargetAngle, uint8_t GoalDistance, int16_t IMU_yaw) {
    bool isRerativeGoalDir = signbit(TargetAngle);
    float _p = (IMU_yaw - TargetAngle) * 0.001;
}
