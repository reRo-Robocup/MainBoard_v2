
/*
 * AttitudeController.cpp
 *
 *  Created on: Feb 15, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "AttitudeController.hpp"

AttitudeController::AttitudeController(baseMcuAbstractionLayer* mcu) {
    _mcu = mcu;
}

void AttitudeController::init() {
    motor_data[TractionMotors::Motor1].pin = MAL::Peripheral_PWM::Motor1;
    motor_data[TractionMotors::Motor1].motorAngle = 45;
    motor_data[TractionMotors::Motor1].isConnectionReserved = false;

    motor_data[TractionMotors::Motor2].pin = MAL::Peripheral_PWM::Motor2;
    motor_data[TractionMotors::Motor2].motorAngle = 135;
    motor_data[TractionMotors::Motor2].isConnectionReserved = false;

    motor_data[TractionMotors::Motor3].pin = MAL::Peripheral_PWM::Motor3;
    motor_data[TractionMotors::Motor3].motorAngle = 225;
    motor_data[TractionMotors::Motor3].isConnectionReserved = true;

    motor_data[TractionMotors::Motor4].pin = MAL::Peripheral_PWM::Motor4;
    motor_data[TractionMotors::Motor4].motorAngle = 315;
    motor_data[TractionMotors::Motor4].isConnectionReserved = false;

    _mode = 0;
}

void AttitudeController::update() {
    switch (_mode) {
        case 0:  // 停止
            _setPWM(TractionMotors::Motor1, 0);
            _setPWM(TractionMotors::Motor2, 0);
            _setPWM(TractionMotors::Motor3, 0);
            _setPWM(TractionMotors::Motor4, 0);
            break;

        case 1:
            _setPWM(TractionMotors::Motor1, 0.5);
            _setPWM(TractionMotors::Motor2, 0.5);
            _setPWM(TractionMotors::Motor3, 0.5);
            _setPWM(TractionMotors::Motor4, 0.5);
            break;

        default:
            break;
    }
}

void AttitudeController::setMode(int mode) {
}

void AttitudeController::_setPWM(TractionMotors motor, float duty) {
    if (motor_data[motor].isConnectionReserved) {
        duty = -duty;
    }
    float lap_duty = (duty + 1) / 2;
    _mcu->pwmSetDuty(motor_data[motor].pin, lap_duty);
}