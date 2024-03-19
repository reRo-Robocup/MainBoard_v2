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
PID <float> PID_LineTrace;

void Keeper::init() {
    PID_RturnGoal.setProcessTime(0.001);
    PID_RturnGoal.setPID(1,0,0);

    PID_LineTrace.setProcessTime(0.001);
    PID_LineTrace.setPID(2,0,0);
}

void Keeper::setLinecenter() {
    if(line->isonLine) {

        uint8_t dis = line->getSensDistance();

        float p = PID_LineTrace.update(120, dis) / 100;

        int8_t dir = abs(line->angle) < 90;

        if(abs(p) > 0.8) {
            p = 0.8;
        }

        int16_t toMove = line->angle + 180;

        atc->setMode(1);
        atc->setGoStraightPower(p);

        atc->setGoStraightAngle(toMove);
        ui->buzzer(1000,1);
    }
}

void Keeper::ReturnGoal() {

}

void Keeper::update() {

    const uint8_t line_dis_threshold = 100;

    if(line->isonLine && cam->KeepGoal.isFront) {
        if(line->getSensDistance() < line_dis_threshold) {
            this->setLinecenter();
        }
        else {
            // int16_t BallAngle_uc = 270 - cam->data.ball_angle;
            // float _x = cos(BallAngle_uc * deg_to_rad);
            atc->setMode(1);
            atc->setGoStraightPower(0.3);
            atc->setGoStraightAngle(cam->data.ball_angle);
        }
    }
}
