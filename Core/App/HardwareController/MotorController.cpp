/*
 * MotorController.cpp
 *
 *  Created on: Dec 23, 2023
 */

#include <math.h>
#include <Devices/Devices.hpp>
#include <HardwareController/MotorController.hpp>

#define deg_to_rad(deg) (((deg) / 360) * 2 * M_PI)
#define rad_to_deg(rad) (((rad) / 2 / M_PI) * 360)

#define STOP_COMPARE 899 / 2

MotorController::MotorController(Devices* devices) {
    _devices = devices;
}

void MotorController::init() {
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor1, 0);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor2, 0);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor3, 0);
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor4, 0);
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
        MotorController::MotorRoll(i,MPowerVector[i]);
    }
}

float MotorController::MotorRoll(int motor, float duty) {
    int Motor_TIM_CH[4] = {
        MAL::Peripheral_PWM::Motor1,
        MAL::Peripheral_PWM::Motor2,
        MAL::Peripheral_PWM::Motor3,
        MAL::Peripheral_PWM::Motor4,
    };
    int _write_compare = 0;
    const float speed_fix = 0.2;
    _write_compare = (duty * speed_fix + 1) * STOP_COMPARE;
    _devices->mcu->pwmSetDuty(MAL::Peripheral_PWM::Motor4, _write_compare);
}