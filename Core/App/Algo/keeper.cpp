/*
 *  keeper.cpp
 *
 *  Created on: 2024/3/14
 *
 *  Author: onlydcx, G4T1PR0
 */

#include <Algo/keeper.hpp>

Keeper::Keeper(MAL* _mcu, AttitudeController* _atc, camera* _cam, KickerController* _kicker, LineSensor* _line, MPU6500* _imu, UI* _ui) {
    this->mcu = _mcu;
    this->atc = _atc;
    this->cam = _cam;
    this->kicker = _kicker;
    this->line = _line;
    this->imu = _imu;
    this->ui = _ui;
}

PID <float> PID_RturnGoal;
PID <float> PID_Trace;

void Keeper::init() {
    PID_RturnGoal.setProcessTime(0.001);
    PID_RturnGoal.setPID(1,0,0);

    PID_Trace.setProcessTime(0.001);
    PID_Trace.setPID(1,0,0);
}

void Keeper::ReturnGoal() {

}


void Keeper::update() {

    const uint8_t goal_dis_min = 0;
    const uint8_t goal_dis_max = 0;

    if(line->isonLine) {
        // トレースできる状態
        // double ball_x = cos(cam->data.ball_angle * deg_to_rad);
        
    }

    if(cam->KeepGoal.isFront) {
        // ゴール前
        // ui->buzzer(1000,1);
        atc->setMode(0);
    } else {
        int8_t dir = signbit(cam->KeepGoal.ang);
        atc->setMode(3);
        atc->setGoStraightPower(0.5);
        if(dir) {
            atc->setGoStraightAngle(90);
        }
        else {
            atc->setGoStraightAngle(-90);
        }
    }
}
