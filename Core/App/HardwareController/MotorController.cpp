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
        MPowerVector[i] = -sin(deg_to_rad(angle - _motorAngles[i]));
        if (MPowerMax < MPowerVector[i])
            MPowerMax = MPowerVector[i];
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

float MotorController::MotorRoll(int motor, float duty) {
    int Motor_TIM_CH[4] = {
        PAL.PWM_CH[MAL::Peripheral_PWM::Motor1],
        PAL.PWM_CH[MAL::Peripheral_PWM::Motor2],
        PAL.PWM_CH[MAL::Peripheral_PWM::Motor3],
        PAL.PWM_CH[MAL::Peripheral_PWM::Motor4],
    };
    int _write_compare = 0;
    const float speed_constant = 0.2;
    _write_compare = (duty * speed_constant + 1); // LAP制御用にDuty変換
    _devices->mcu->pwmSetDuty(Motor_TIM_CH[i], _write_compare);
}