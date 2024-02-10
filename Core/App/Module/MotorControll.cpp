/*
 * MotorControll.cpp
 *
 *  Created on: Dec 23, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#include "MotorControll.hpp"

const bool isMotorPinReversed[4] = {
    false, false, true, true
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
        // this->_write_duty(i, 1);
        _mcu->pwmSetDuty(motor[i], 0.5);
    }
    // for(int i = 0; i < 4; i++) {
    //     _mcu->pwmSetDuty(motor[i], 1.0);
    // }
    // _mcu->delay_ms(100000);
    // for(int i = 0; i < 4; i++) {
    //     _mcu->pwmSetDuty(motor[i], 0.5);
    // }
}

float MotorControll::_getBatteryVoltage() {
    return _mcu->adcGetValue(MAL::Peripheral_ADC::BatteryVoltage) / 4096;
}

float MotorControll::_duty_to_LAPduty(float duty) {
    return ((duty + 1) / 2);
}

bool MotorControll::isDRVsleep() {
    return _mcu->gpioGetValue(MAL::Peripheral_GPIO::isMotorEnabled);
}

void MotorControll::run(int16_t angle) {

    angle = 450 - angle;

    while(angle >= 360)  angle -= 360;
    while(angle <= -360) angle += 360;

    printf("angle : %d\n", angle);

    float MPowerVector[4] = {0};  // 4つのモーターの出力比
    float MPowerMax = 0;          // 最大値

    for (int i = 0; i < 4; i++) {
        float tmp_angle = (angle - _motorAngles[i]) * (M_PI / 180);
        MPowerVector[i] = cos(tmp_angle);
        printf("%f ", MPowerVector[i]);

        // if (MPowerMax < MPowerVector[i])
        //     MPowerMax = MPowerVector[i];
        // if (isMotorPinReversed[i])
        //     MPowerVector[i] *= -1;
    }

    // if ((MPowerMax = !1) || (MPowerMax = !-1)) {
    //     for (int i = 0; i < 4; i++) {
    //         MPowerVector[i] *= (1 / MPowerMax);
    //     }
    // }

    float _write_compare[4] = {0};

    for (int i = 0; i < 4; i++) {
        _write_compare[i] = MotorControll::_duty_to_LAPduty(MPowerVector[i]);
    }

    // char tmp[64];
    // sprintf(tmp, "%f %f %f %f", _write_compare[0],_write_compare[1],_write_compare[2],_write_compare[3]);
    // printf("%s\n",tmp);

    printf("\n");

    for(int i = 0; i < 4; i++) {
        _mcu->pwmSetDuty(motor[i], _write_compare[i]);
    }
}

void MotorControll::turn() {
    // static int16_t _prev_yaw = 0;
    // int16_t angle_diff = targetAgle - yaw;
    // int16_t prev_diff = _prev_yaw - yaw;
    // int8_t dir = signbit(angle_diff)? 1: -1;
    // float kp, ki, kd;
    // float _speed = kp * angle_diff + ki + kd;
    // _prev_yaw = yaw;
    for(int i = 0; i < 4; i++) {
        // this->_write_duty(motor[i], 0);
    }
}

void MotorControll::carryBall(int16_t TargetAngle, uint8_t GoalDistance, int16_t IMU_yaw) {
    bool isRerativeGoalDir = signbit(TargetAngle);
    float _p = (IMU_yaw - TargetAngle) * 0.001;
}

int16_t _prev_Vx, _prev_Vy;

void MotorControll::approach_Ball(int16_t BallAngle, uint16_t BallDistance,  int16_t IMU_yaw, int16_t IMU_Vx, int16_t IMU_Vy) {
    float dt = CONTROLL_CYCLE;
    float kp[2], ki[2], kd[2];
    int16_t mAngle = 0;

    mAngle = 450 - mAngle;
    this->map2_180(mAngle);
    float _mAngle_xy[2] = {0.0};
    _mAngle_xy[0] = cos(rad_to_deg(mAngle)); // x成分
    _mAngle_xy[1] = sin(rad_to_deg(mAngle)); // y成分

    // kp 算出
    kp[0] = _prev_Vx - IMU_Vx;
    kp[1] = _prev_Vy - IMU_Vy;

    // Ki 算出

    _prev_Vx = IMU_Vx;
    _prev_Vy = IMU_Vy; 
}


void MotorControll::roll(float M1duty, float M2duty, float M3duty, float M4duty) {
    M1duty = this->_duty_to_LAPduty(M1duty);
    M2duty = this->_duty_to_LAPduty(M2duty);

    M3duty *= -1;

    M3duty = this->_duty_to_LAPduty(M3duty);
    M4duty = this->_duty_to_LAPduty(M4duty);

    _mcu->pwmSetDuty(motor[0], M1duty);
    _mcu->pwmSetDuty(motor[1], M2duty);
    _mcu->pwmSetDuty(motor[2], M3duty);
    _mcu->pwmSetDuty(motor[3], M4duty);
}