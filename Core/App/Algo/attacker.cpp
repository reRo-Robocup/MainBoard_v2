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

int16_t Ball_Angle;
int16_t BallDis;
int16_t toMove;

void Attacker::update() {
    atc->setGoStraightPower(0.9);
    
    Ball_Angle = cam->data.ball_angle - 170;
    BallDis = cam->data.ball_distance;

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

            if(abs(Ball_Angle) <= 10) {
                // atc->setGoStraightAngle(0);
                toMove = 0;
            }
            else {
                if(BallDis > 100) {
                    // atc->setGoStraightAngle(Ball_Angle);
                    toMove = Ball_Angle;
                }
                else {
                    int8_t dir = signbit(Ball_Angle);
                    if(dir) {
                        toMove = Ball_Angle + 30;
                        // atc->setGoStraightAngle(Ball_Angle + 30);
                    }
                    else {
                        toMove = Ball_Angle - 30;
                        // atc->setGoStraightAngle(Ball_Angle - 30);
                    }
                }
            }

            atc->setGoStraightAngle(toMove);
        }
    }
}