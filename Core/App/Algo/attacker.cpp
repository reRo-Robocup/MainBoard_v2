/*
 *  attacker.cpp
 *
 *  Created on: 2024/3/14
 *
 *  Author: onlydcx, G4T1PR0
 */

#include <Algo/attacker.hpp>

Attacker::Attacker(MAL* mcu, AttitudeController* atc, camera* cam, KickerController* kicker, LineSensor* line, MPU6500* imu, UI* ui) {
    this->_mcu = mcu;
    this->_atc = atc;
    this->_cam = cam;
    this->_kicker = kicker;
    this->_line = line;
    this->_imu = imu;
    this->_ui = ui;
}

void Attacker::init() {
}

void Attacker::update() {
}