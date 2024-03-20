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

PID <float> PID_ReturnGoal;
PID <float> PID_LineBack;
PID <float> PID_BallMoveingX;

void Keeper::init() {
    PID_ReturnGoal.setProcessTime(0.001);
    PID_ReturnGoal.setPID(2,0,0);

    PID_LineBack.setProcessTime(0.001);
    PID_LineBack.setPID(2,0,0);

    PID_BallMoveingX.setProcessTime(0.001);
    PID_BallMoveingX.setPID(2.5,0,0.2);
}

void Keeper::setLinecenter() {
    if(line->isonLine) {

        uint8_t dis = line->getSensDistance();

        float p = PID_LineBack.update(120, dis) / 100;

        int8_t dir = abs(line->angle) < 90;

        if(abs(p) > 0.7) {
            p = 0.7;
        }

        int16_t toMove = line->angle + 180;

        atc->setMode(3);
        atc->setGoStraightPower(p);

        atc->setGoStraightAngle(toMove);
        ui->buzzer(1000,1);
    }
}

float rt_power;

void Keeper::ReturnGoal() {
    uint8_t goal_distance_threshold = 90;
    if(cam->KeepGoal.dis > goal_distance_threshold) {
        rt_power = PID_ReturnGoal.update(goal_distance_threshold, cam->KeepGoal.dis);
        rt_power /= 150;
        if(abs(rt_power > 0.8)) rt_power = 0.8;

        // todo
        // 復帰時にオウンゴール対策

        atc->setMode(3);
        atc->setGoStraightAngle(cam->KeepGoal.ang + 180);
        atc->setGoStraightPower(abs(rt_power));
    }
}

float _ball_x;
int16_t BallAngle;
int16_t BallAngle_uc;
float power;
bool dir;

void Keeper::update() {

    ReturnGoal();

    // const uint8_t line_dis_threshold = 100;

    // BallAngle = cam->data.ball_angle;

    // if(line->isonLine && cam->KeepGoal.isFront) {
    //     if(line->getSensDistance() < line_dis_threshold) {
    //         this->setLinecenter();
    //     }
    //     else {  
            
    //         BallAngle_uc = 270 - BallAngle;
            
    //         _ball_x = cos(BallAngle_uc * deg_to_rad);

    //         power = PID_BallMoveingX.update(0, _ball_x);

    //         atc->setMode(3);
    //         atc->setGoStraightPower(abs(power));
    //         // atc->setGoStraightAngle(cam->data.ball_angle);
    //         // int8_t dir = signbit(BallAngle);
    //         dir = (BallAngle - 180) > 0;
    //         if(dir) atc->setGoStraightAngle(90);
    //         else atc->setGoStraightAngle(-90);
    //     }
    // }
    // else if (line->isonLine && !cam->KeepGoal.isFront) {
    //     bool dir = signbit(cam->KeepGoal.ang);
    //     atc->setMode(3);
    //     atc->setGoStraightPower(0.6);
    //     if(dir) atc->setGoStraightAngle(45);
    //     else atc->setGoStraightAngle(-45);
    // }
    // else {
    //     // atc->setMode(0);
    //     this->ReturnGoal();
    // }
}
