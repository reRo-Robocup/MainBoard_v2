
/*
 * AttitudeController.cpp
 *
 *  Created on: Feb 15, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "AttitudeController.hpp"

AttitudeController::AttitudeController(baseMcuAbstractionLayer* mcu, MPU6500* imu) {
    _mcu = mcu;
    _imu = imu;
}

void AttitudeController::init() {
    motor_data[TractionMotors::Motor1].pin = MAL::Peripheral_PWM::Motor1;
    motor_data[TractionMotors::Motor1].motorAngle = 45;
    motor_data[TractionMotors::Motor1].isConnectionReversed = false;

    motor_data[TractionMotors::Motor2].pin = MAL::Peripheral_PWM::Motor2;
    motor_data[TractionMotors::Motor2].motorAngle = 135;
    motor_data[TractionMotors::Motor2].isConnectionReversed = false;

    motor_data[TractionMotors::Motor3].pin = MAL::Peripheral_PWM::Motor3;
    motor_data[TractionMotors::Motor3].motorAngle = 225;
    motor_data[TractionMotors::Motor3].isConnectionReversed = true;

    motor_data[TractionMotors::Motor4].pin = MAL::Peripheral_PWM::Motor4;
    motor_data[TractionMotors::Motor4].motorAngle = 315;
    motor_data[TractionMotors::Motor4].isConnectionReversed = false;

    // _turn_angle_pid.setPID(0.02, 0.0, 0);
    _turn_angle_pid.setPID(0.005, 0.0, 0);
    _mode = 0;
    this->_turn_angle = 180;
    this->setGoStraightPower(0.5);
}

void AttitudeController::update() {
    switch (_mode) {
        case 0:  // 停止
            _setPWM(TractionMotors::Motor1, 0);
            _setPWM(TractionMotors::Motor2, 0);
            _setPWM(TractionMotors::Motor3, 0);
            _setPWM(TractionMotors::Motor4, 0);
            break;

        case 1:  // 移動のみ
        {
            _go_straight_angle += 90;
            _go_straight_angle = 450 - _go_straight_angle;
            while (_go_straight_angle >= 360)
                _go_straight_angle -= 360;
            while (_go_straight_angle <= -360)
                _go_straight_angle += 360;
            float MPowerVector[4] = {0};  // 4つのモーターの出力比
            float MPowerMax = 0;          // 最大値

            for (int i = 0; i < 4; i++) {
                float tmp_angle = (_go_straight_angle - motor_data[i].motorAngle) * (M_PI / 180);
                MPowerVector[i] = cos(tmp_angle);
                // printf("%f ", MPowerVector[i]);

                if (MPowerMax < MPowerVector[i])
                    MPowerMax = MPowerVector[i];
            }

            if ((MPowerMax != 1) || (MPowerMax != -1)) {
                for (int i = 0; i < 4; i++) {
                    MPowerVector[i] *= (_go_straight_power / MPowerMax);
                }
            }

            _setPWM(TractionMotors::Motor1, MPowerVector[0]);
            _setPWM(TractionMotors::Motor2, MPowerVector[1]);
            _setPWM(TractionMotors::Motor3, MPowerVector[2]);
            _setPWM(TractionMotors::Motor4, MPowerVector[3]);
        } break;

        case 2:  // 角度制御のみ
        {
            float output = _turn_angle_pid.update(_turn_angle, _imu->Yaw);
            if (output > 0.94) {
                output = 0.94;
            }
            if (output < -0.94) {
                output = -0.94;
            }
            _setPWM(TractionMotors::Motor1, output);
            _setPWM(TractionMotors::Motor2, output);
            _setPWM(TractionMotors::Motor3, output);
            _setPWM(TractionMotors::Motor4, output);
        } break;

        case 3:  // 角度制御あり移動
        {
            _go_straight_angle += 90;
            _go_straight_angle = 450 - _go_straight_angle;
            while (_go_straight_angle >= 360)
                _go_straight_angle -= 360;
            while (_go_straight_angle <= -360)
                _go_straight_angle += 360;
            float MPowerVector[4] = {0};  // 4つのモーターの出力比
            float MPowerMax = 0;          // 最大値

            for (int i = 0; i < 4; i++) {
                float tmp_angle = (_go_straight_angle - motor_data[i].motorAngle) * (M_PI / 180);
                MPowerVector[i] = cos(tmp_angle);
                // printf("%f ", MPowerVector[i]);

                if (MPowerMax < abs(MPowerVector[i]))
                    MPowerMax = abs(MPowerVector[i]);
            }

            if ((MPowerMax != 1) || (MPowerMax != -1)) {
                for (int i = 0; i < 4; i++) {
                    MPowerVector[i] *= (_go_straight_power / MPowerMax);
                }
            }

            float output = _turn_angle_pid.update(_turn_angle, _imu->Yaw);

            if (output > 0.94) {
                output = 0.94;
            }
            if (output < -0.94) {
                output = -0.94;
            }

            if (abs(output) > 0.05) {
                _setPWM(TractionMotors::Motor1, output);
                _setPWM(TractionMotors::Motor2, output);
                _setPWM(TractionMotors::Motor3, output);
                _setPWM(TractionMotors::Motor4, output);
            } else {
                for (int i = 0; i < 4; i++) {
                    MPowerVector[i] += output;
                    if (MPowerMax < abs(MPowerVector[i]))
                        MPowerMax = abs(MPowerVector[i]);
                }

                if ((MPowerMax != 1)) {
                    for (int i = 0; i < 4; i++) {
                        MPowerVector[i] *= (0.94 / MPowerMax);
                    }
                }

                _setPWM(TractionMotors::Motor1, MPowerVector[0]);
                _setPWM(TractionMotors::Motor2, MPowerVector[1]);
                _setPWM(TractionMotors::Motor3, MPowerVector[2]);
                _setPWM(TractionMotors::Motor4, MPowerVector[3]);
            }

        } break;

        default:
            _setPWM(TractionMotors::Motor1, 0);
            _setPWM(TractionMotors::Motor2, 0);
            _setPWM(TractionMotors::Motor3, 0);
            _setPWM(TractionMotors::Motor4, 0);
            break;
    }
}

void AttitudeController::setMode(int mode) {
    _mode = mode;
}

void AttitudeController::setGoStraightAngle(int16_t angle) {
    _go_straight_angle = angle;
}

void AttitudeController::setGoStraightPower(float power) {
    _go_straight_power = power;
}

void AttitudeController::setTurnAngle(int angle) {
    _turn_angle = angle;
}

void AttitudeController::_setPWM(TractionMotors motor, float duty) {
    if (motor_data[motor].isConnectionReversed) {
        duty = -duty;
    }
    float lap_duty = (duty + 1) / 2;
    _mcu->pwmSetDuty(motor_data[motor].pin, lap_duty);
}