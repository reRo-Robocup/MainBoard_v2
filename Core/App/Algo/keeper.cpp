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

void Keeper::init() {
    PID_ReturnGoal_X.setProcessTime(0.001);
    PID_ReturnGoal_X.setPID(0.04, 0, 0);

    PID_ReturnGoal_Y.setProcessTime(0.001);
    PID_ReturnGoal_Y.setPID(0.04, 0, 0);

    PID_LineBack.setProcessTime(0.001);
    PID_LineBack.setPID(2, 0, 0);

    PID_GuardGoal.setProcessTime(0.001);
    PID_GuardGoal.setPID(0.04, 0, 0);

    PID_traceBallY.setProcessTime(0.001);
    PID_traceBallY.setPID(0.04, 0, 0);
}

void Keeper::update() {
    _mode = 1;
    switch (_mode) {
        case 0:

            break;
        case 1:
            _returnGoal();
            if (cam->KeepGoal.dis < _goal_target + 10) {
                _mode = 2;
            }
            break;
        case 2:
            _guardGoal();
            if (cam->KeepGoal.dis > _goal_target + 15) {
                _mode = 1;
            }
            break;
        default:
            break;
    }
}

void Keeper::_setLinecenter() {
    if (line->isonLine) {
        uint8_t dis = line->getSensDistance();

        float p = PID_LineBack.update(120, dis) / 100;

        // int8_t dir = abs(line->angle) < 90;

        if (abs(p) > 0.7) {
            p = 0.7;
        }

        int16_t toMove = line->angle + 180;

        atc->setMode(3);
        atc->setGoStraightPower(p);

        atc->setGoStraightAngle(toMove);
        ui->buzzer(1000, 1);
    }
}

void Keeper::_returnGoal() {
    float temp_2 = 180 - cam->KeepGoal.ang;
    float observed_goal_x = cos((temp_2)*deg_to_rad) * cam->KeepGoal.dis;
    float observed_goal_y = sin((temp_2)*deg_to_rad) * cam->KeepGoal.dis;

    float out_x = PID_ReturnGoal_X.update(0, observed_goal_x) * -1;
    float out_y = PID_ReturnGoal_Y.update(_goal_target, observed_goal_y);

    ui->buzzer(600, 10);
    // printf("obs_x: %f obs_Y %f out_x %f out_y %f\r\n", observed_goal_x, observed_goal_y, out_x, out_y);

    if (out_x > 0.94) {
        out_x = 0.94;
    } else if (out_x < -0.94) {
        out_x = -0.94;
    }

    if (out_y > 0.94) {
        out_y = 0.94;
    } else if (out_y < -0.94) {
        out_y = -0.94;
    }

    atc->setMode(4);
    atc->setGoStraightXY(out_x, out_y);
}

void Keeper::_guardGoal() {
    float _ball_angle = 90 + cam->data.ball_angle;

    float temp_1 = 180 - _ball_angle;
    float observed_ball_x = cos((temp_1)*deg_to_rad) * cam->data.ball_distance;
    float observed_ball_y = sin((temp_1)*deg_to_rad) * cam->data.ball_distance;

    float out_x = PID_GuardGoal.update(0, observed_ball_x);

    float temp_2 = 180 - cam->KeepGoal.ang;
    float observed_goal_y = sin((temp_2)*deg_to_rad) * cam->KeepGoal.dis;

    float out_y_ball = PID_traceBallY.update(_ball_distance_target, observed_ball_y) * -1;
    float out_y_goal = PID_ReturnGoal_Y.update(_goal_target, observed_goal_y);

    float out_y = out_y_goal;

    if (line->isonLine) {
        // if (0) {
        atc->setMode(3);
        atc->setGoStraightAngle(line->angle);
    } else {
        if (!cam->KeepGoal.isFront) {
            out_y = out_y_ball;
            ui->buzzer(2000, 10);
        }

        if (!cam->data.isBallDetected) {
            // if (0) {
            out_x = 0;
            out_y = 0;
            ui->buzzer(1000, 10);
        }
        ui->setLED(1, cam->data.isBallDetected);

        // printf("is_front: %d, goal_angle: %d, ball_angle: %d, distance: %d\r\n", cam->KeepGoal.isFront, cam->KeepGoal.ang, cam->data.ball_angle, cam->KeepGoal.dis);
        // printf("ba: %f is_f: %d out_x: %f out_y: %f goal_d: %d ball_d: %d\r\n", _ball_angle, cam->KeepGoal.isFront, out_x, out_y, cam->KeepGoal.dis, cam->data.ball_distance);

        if (out_x > 0.94) {
            out_x = 0.94;
        } else if (out_x < -0.94) {
            out_x = -0.94;
        }

        if (out_y > 0.94) {
            out_y = 0.94;
        } else if (out_y < -0.94) {
            out_y = -0.94;
        }

        atc->setMode(4);
        atc->setGoStraightXY(out_x, out_y);
    }

    // printf("observed_x: %f, out_x: %f\r\n", observed_x, out_x);
}
