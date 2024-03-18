/*
 *  attacker.cpp
 *
 *  Created on: 2024/3/14
 *
 *  Author: onlydcx, G4T1PR0
 */

#include <Algo/attacker.hpp>

Attacker::Attacker(MAL* _mcu, AttitudeController* _atc, camera* _cam, KickerController* _kicker, LineSensor* _line, MPU6500* _imu, UI* _ui) {
    this->mcu = _mcu;
    this->atc = _atc;
    this->cam = _cam;
    this->kicker = _kicker;
    this->line = _line;
    this->imu = _imu;
    this->ui = _ui;
}

void Attacker::init() {
}

void Attacker::update() {
    atc->setGoStraightPower(0.9);
    if(line->isonLine) {
        // ライン上
        atc->setMode(3);
        atc->setGoStraightAngle(line->angle);
        // ui->buzzer(1000,1);
    }
    else {
        if(!cam->data.isBallDetected) {
            atc->setMode(0);
            ui->buzzer(1000,1);
        }
        else {
            atc->setMode(3);
            int16_t BallAngle = cam->data.ball_angle;
            uint8_t distance = cam->data.ball_distance;

            if(abs(BallAngle) <= 10) {
                atc->setGoStraightAngle(0);
            }
            else {
                if(distance > 50) {
                    atc->setGoStraightAngle(BallAngle);
                }
                else {
                    int8_t dir = signbit(BallAngle);
                    if(dir) {
                        atc->setGoStraightAngle(BallAngle + 30);
                    }
                    else {
                        atc->setGoStraightAngle(BallAngle - 30);
                    }
                }
            }
        }
    }
}