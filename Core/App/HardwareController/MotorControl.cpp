/*
 * lineSensorAlgorithm.cpp
 *
 *  Created on: Dec 23, 2023
 */

#include <math.h>
#include <Devices/Devices.hpp>
#include <HardwareController/MotorControl.hpp>

#define deg_to_rad(deg) (((deg) / 360) * 2 * M_PI)
#define rad_to_deg(rad) (((rad) / 2 / M_PI) * 360)

MotorContol::MotorControl(Devices* devices) {
    _devices = devices;
}

void MotorControl::init() {
    pwmSetDuty(Motor1, 0);
    pwmSetDuty(Motor2, 0);
    pwmSetDuty(Motor3, 0);
    pwmSetDuty(Motor4, 0);
}

void MotorContol::run(uint8_t angle, uint8_t speed) {
    angle = 450 - angle;
    while(angle > 359) angle -= 360;
    while(angle < 0)   angle += 360;
    float MPowerVector[4] = {0}; // 4つのモーターの出力比
    float MPowerMax = 0; // 最大値
    for(int i = 0; i < 4; i++) {
        MPowerVector[i] = -sin(deg_to_rad(angle - _motorAngles[i]));
        if(MPwrMax < MPwrVector[i]) MPwrMax = MPwrVector[i];
    }
    if(MPwrMax < 1) {
        for(int i = 0; i < 4; i++) {
            MPwrVector[i] *= (1 / MPwrMax);
        }
    }
    for(int i = 0; i < 4; i++) {
        pwmSetDuty(Motor1, MPwrVector[0]);
        pwmSetDuty(Motor2, MPwrVector[1]);
        pwmSetDuty(Motor3, MPwrVector[2]);
        pwmSetDuty(Motor4, MPwrVector[3]);
    }
}