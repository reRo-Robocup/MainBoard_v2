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
    PID_ReturnGoal_X.setPID(0.8, 0, 0);

    PID_ReturnGoal_Y.setProcessTime(0.001);
    PID_ReturnGoal_Y.setPID(0.04, 0, 0);

    PID_LineBack.setProcessTime(0.001);
    PID_LineBack.setPID(2, 0, 0);

    PID_GuardGoal.setProcessTime(0.001);
    PID_GuardGoal.setPID(2.9, 0, 0);

    PID_traceBallY.setProcessTime(0.001);
    PID_traceBallY.setPID(0.04, 0, 0);
}

void Keeper::update() {
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
    float observed_x = cos((90 - cam->KeepGoal.ang) * deg_to_rad);

    float out_x = PID_ReturnGoal_X.update(0, observed_x) * -1;
    float out_y = PID_ReturnGoal_Y.update(_goal_target, cam->KeepGoal.dis);

    ui->buzzer(600, 10);

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
    float observed_x = cos((90 - cam->data.ball_angle) * deg_to_rad);

    float _corrected_goal_distance_ang = 180 - abs(cam->KeepGoal.ang);
    float _corrected_goal_distance = int(cos(_corrected_goal_distance_ang * deg_to_rad) * cam->KeepGoal.dis);

    float out_x = PID_GuardGoal.update(0, observed_x) * -1;

    float out_y_ball = PID_traceBallY.update(_ball_distance_target, cam->data.ball_distance) * -1;
    float out_y_goal = PID_ReturnGoal_Y.update(_goal_target, cam->KeepGoal.dis);

    float out_y = out_y_goal;
    // if (cam->data.ball_distance > _ball_distance_target) {

    // if (cam->data.isBallDetected) {
    //     if (cam->data.ball_distance < 60) {
    //         ui->buzzer(1500, 10);
    //         if (abs(cam->data.ball_angle - _prev_ball_angle) > 4 || abs(_prev_ball_distance - cam->data.ball_distance) > 4) {
    //             _ball_move_check_time = mcu->millis();
    //             ui->buzzer(600, 10);
    //         } else {
    //             if (mcu->millis() - _ball_move_check_time > 2000) {
    //                 ui->buzzer(2400, 10);
    //             }
    //         }
    //         _prev_ball_angle = cam->data.ball_angle;
    //         _prev_ball_distance = cam->data.ball_distance;
    //     }
    // } else {
    //     _ball_move_check_time = mcu->millis();
    //     ui->buzzer(700, 10);
    // }

    if (line->isonLine) {
        // if (0) {
        atc->setMode(3);
        atc->setGoStraightAngle(line->angle);
    } else {
        if (!cam->KeepGoal.isFront) {
            out_y = out_y_ball;
            ui->buzzer(2000, 10);
            // if (cam->KeepGoal.ang > 0) {                                         // マシン位置判定 左端
            //     if (cam->data.ball_angle < 360 && cam->data.ball_angle > 200) {  // ボールがマシンより左
            //         // out_x = 0;
            //         out_y = out_y_ball;
            //         ui->buzzer(2000, 10);
            //     } else {
            //         out_y = out_y_ball;
            //     }

            // } else {                                                           // マシン位置判定 右端
            //     if (cam->data.ball_angle > 0 && cam->data.ball_angle < 160) {  // ボールがマシンより右
            //         // out_x = 0;
            //         out_y = out_y_ball;
            //         ui->buzzer(2000, 10);
            //     } else {
            //         out_y = out_y_ball;
            //     }
            // }
        }

        if (!cam->data.isBallDetected) {
            out_x = 0;
            out_y = 0;
            ui->buzzer(800, 10);
        }
        ui->setLED(1, cam->data.isBallDetected);

        // printf("is_front: %d, goal_angle: %d, ball_angle: %d, distance: %d\r\n", cam->KeepGoal.isFront, cam->KeepGoal.ang, cam->data.ball_angle, cam->KeepGoal.dis);
        // printf("is_f: %d out_x: %f out_y: %f goal_d: %d ball_d: %d\r\n", cam->KeepGoal.isFront, out_x, out_y, cam->KeepGoal.dis, cam->data.ball_distance);

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
