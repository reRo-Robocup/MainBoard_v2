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

PID <float> PID_AttX;
PID <float> PID_AttY;

void Attacker::init() {
    PID_AttX.setProcessTime(0.001);
    PID_AttX.setPID(2,0,0.2);

    PID_AttY.setProcessTime(0.001);
    PID_AttY.setPID(2,0,0);
}

int16_t Ball_Angle;
int16_t BallDis;
int16_t toMove;

float moving_x;
float moving_y;

float _ball_xvect;
float _ball_yvect;

float target_y;

int16_t BallAngle_uc;

void Attacker::update() {
    atc->setGoStraightPower(0.7);
    Ball_Angle = cam->data.ball_angle - 180;
    BallDis = cam->data.ball_distance;

    if(line->isonLine) {
        /* TODO: xy分解 */
        atc->setMode(3);
        atc->setGoStraightAngle(line->angle);
    }
    else {
        if(!cam->data.isBallDetected) {
            atc->setMode(0);
        }
        else {
            if (BallDis > 50) {
                toMove = Ball_Angle;
                if(line->isonLine) {
                    
                }
                atc->setMode(3);
                atc->setGoStraightAngle(toMove);
            }
            else {

                BallAngle_uc = 270 - Ball_Angle;

                // X軸 PID
                _ball_xvect = cos(BallAngle_uc * deg_to_rad);
                moving_x = PID_AttX.update(0, _ball_xvect) * -1;

                if(moving_x > 0.94) {
                    moving_x = 0.94;
                }
                else if (moving_x < -0.94) {
                    moving_x = -0.94;
                }

                // Y軸 PID

                _ball_yvect = sin(BallAngle_uc * deg_to_rad);

                uint8_t u_ball_angle = abs(Ball_Angle);
                int8_t dir = signbit(Ball_Angle)? -1: 1;

                if(u_ball_angle <= 10) {
                    toMove = 0;
                }
                else {
                    toMove = Ball_Angle + (30 * dir);
                }
                // else if(u_ball_angle <= 45) {
                //     toMove = 90 * dir;
                // }
                // else {
                //     toMove = (u_ball_angle + 50)  * dir;
                // }

                target_y = sin((270 - toMove) * deg_to_rad);
                moving_y = PID_AttY.update(target_y, _ball_yvect) * -1;

                if(moving_y > 0.94) {
                    moving_y = 0.94;
                }
                else if (moving_y < -0.94) {
                    moving_y = -0.94;
                }

                atc->setMode(4);
                atc->setGoStraightXY(moving_x, moving_y);
            }
        }
    }
}