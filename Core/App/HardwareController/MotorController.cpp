/*
 * MotorController.cpp
 *
 *  Created on: Dec 23, 2023
 */

#include <math.h>
#include <Devices/Devices.hpp>
#include <HardwareController/MotorController.hpp>
#include "GlobalDefines.h"

#define MOTOR_STOP_COMPARE (__HAL_TIM_GET_AUTORELOAD(&tim1) / 2)

const bool isMotorPinReversed[4] = {
    // モーターの配線逆か？
    false, false, false, false
};

const int Motor_TIM_CH[4] = {
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor1],
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor2],
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor3],
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor4],
};

const float speed_constant = 0.2;
const uint8_t _motorAngles[4] = {45, 135, 225, 315};  // モーターの配置角度

MotorController::MotorController(Devices* devices) {
    _devices = devices;
}

void MotorController::init() {
    for(int i = 0; i < 4; i++) {
        MotorController::MotorRoll(i, MOTOR_STOP_COMPARE);
    }
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
        if(isMotorPinReversed)
            MPowerVector[i] *= -1;
    }
    if (MPowerMax =! 1 || MPowerMax =! -1) {
        for (int i = 0; i < 4; i++) {
            MPowerVector[i] *= (1 / MPowerMax);
        }
    }
    for (int i = 0; i < 4; i++) {
        MotorController::MotorRoll(i, MPowerVector[i]);
    }
}

void MotorController::MotorRoll(int motor, float duty) {
    int _write_compare = ((duty * speed_constant + 1) / 2); // LAP制御用にDuty変換
    _devices->mcu->pwmSetDuty(Motor_TIM_CH[i], _write_compare);
}