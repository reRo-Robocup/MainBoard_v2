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
    if(line->isonLine) {
        // ライン上
        atc->setMode(3);
        atc->setGoStraightAngle(line->angle);
        ui->buzzer(1000,1);
    }
    else {
        if(cam->data.isBallDetected) {
            // ボール見つからない
            if(cam->data.isInMyArea) {
                // 待機
                atc->setMode(0);
            }
            else {
                
            }
        }
    }
}