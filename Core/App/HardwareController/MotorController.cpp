/*
 * MotorController.cpp
 *
 *  Created on: Dec 23, 2023
 */

#include <math.h>
#include <Devices/Devices.hpp>
#include <HardwareController/MotorController.hpp>
#include "GlobalDefines.h"

#define MOTOR_STOP_COMPARE (__HAL_TIM_GET_AUTORELOAD(&htim1) / 2)

const bool isMotorPinReversed[4] = {
    false, false, false, false
};

const float speed_constant = 0.2;
const uint16_t _motorAngles[4] = {45, 135, 225, 315};  // モーターの配置角度

MotorController::MotorController(Devices* devices) {
    _devices = devices;
}

void MotorController::init() {
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor1, 0.5);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor2, 0.5);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor3, 0.5);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor4, 0.5);
}

void MotorController::run(uint8_t angle, uint8_t speed) {
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
        _write_compare[i] = ((MPowerVector[i] * speed_constant + 1) / 2);
    }
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor1, _write_compare[0]);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor2, _write_compare[1]);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor3, _write_compare[2]);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor4, _write_compare[3]);
}

void MotorController::turn(bool cw, uint8_t speed) {
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor1, speed);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor2, speed);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor3, speed);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor4, speed);
}