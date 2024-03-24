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
    PID_traceBallX.setProcessTime(0.001);
    PID_traceBallX.setPID(0.04, 0, 0);

    PID_traceBallY.setProcessTime(0.001);
    PID_traceBallY.setPID(0.04, 0, 0);
}

void Attacker::update2() {
    float observed_x = cos((90 - cam->data.ball_angle) * deg_to_rad);

    float out_x = PID_traceBallX2.update(0, observed_x) * -1;
    float out_y = PID_traceBallY2.update(0, cam->KeepGoal.dis);

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

void Attacker::update() {
    float _ball_angle = 90 + cam->data.ball_angle;
    if (_ball_angle > 360) {
        _ball_angle -= 360;
    }

    int ball_pos = 0;
    if (_ball_angle < 360 && _ball_angle > 270) {
        ball_pos = 1;  // 左うしろ
    } else if (_ball_angle > 180 && _ball_angle < 270) {
        ball_pos = 2;  // 右うしろ
    } else {
        ball_pos = 0;  // 前方
    }

    float _ball_distance = 0;
    if (ball_pos == 1 || ball_pos == 2) {
        _ball_distance = -cam->data.ball_distance;
    } else {
        _ball_distance = cam->data.ball_distance;
    }
    float temp_1 = 180 - _ball_angle;
    float observed_x = cos((temp_1)*deg_to_rad) * cam->data.ball_distance;
    float observed_y = sin((temp_1)*deg_to_rad) * cam->data.ball_distance;

    float out_x = 0;
    float out_y = 0;

    // if (!cam->data.isBallDetected) {
    //     _mode = 0;
    // }
    _mode = 0;

    switch (_mode) {
        case 0:  // ボール前方
            out_x = PID_traceBallX.update(0, observed_x) * -1;
            out_y = PID_traceBallY.update(0, observed_y) * -1;
            ui->buzzer(500, 10);

            // if (ball_pos == 1) {
            //     _mode = 10;
            // } else if (ball_pos == 2) {
            //     _mode = 20;
            // }
            break;

        case 10:  // ボール左うしろ
            out_x = PID_traceBallX.update(-40, observed_x) * -1;
            out_y = PID_traceBallY.update(50, observed_y) * -1;
            ui->buzzer(1000, 10);

            if (ball_pos == 0) {
                _mode = 11;
            }
            break;

        case 11:  // ボール前方
            out_x = PID_traceBallX.update(-40, observed_x) * -1;
            out_y = PID_traceBallY.update(50, observed_y) * -1;
            ui->buzzer(1500, 10);

            if (observed_y > 20) {
                _mode = 12;
            }

        case 12:
            out_x = PID_traceBallX.update(-40, observed_x) * -1;
            out_y = PID_traceBallY.update(50, observed_y) * -1;
            ui->buzzer(2000, 10);

            if (observed_y > 20) {
                _mode = 0;
            }
            break;

        case 20:  // ボール右うしろ
            out_x = PID_traceBallX.update(40, observed_x) * -1;
            out_y = PID_traceBallY.update(50, observed_y) * -1;
            ui->buzzer(500, 10);

            if (ball_pos == 0) {
                _mode = 21;
            }
            break;

        case 21:  // ボール前方
            out_x = PID_traceBallX.update(40, observed_x) * -1;
            out_y = PID_traceBallY.update(50, observed_y) * -1;
            ui->buzzer(1000, 10);

            if (observed_y > 20) {
                _mode = 22;
            }
            break;

        case 22:
            out_x = PID_traceBallX.update(40, observed_x) * -1;
            out_y = PID_traceBallY.update(50, observed_y) * -1;
            ui->buzzer(1000, 10);

            if (observed_y > 20) {
                _mode = 0;
            }
            break;

        default:
            _mode = 0;
            break;
    }

    if (line->isonLine) {
        // if (0) {
        atc->setMode(3);
        atc->setGoStraightAngle(line->angle);
        ui->buzzer(2000, 10);
    } else {
        if (!cam->data.isBallDetected) {
            // if (0) {
            out_x = 0;
            out_y = 0;
            ui->buzzer(1000, 10);
        }
        ui->setLED(1, cam->data.isBallDetected);

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

        // printf("is_front: %d, goal_angle: %d, ball_angle: %d, distance: %d\r\n", cam->KeepGoal.isFront, cam->KeepGoal.ang, cam->data.ball_angle, cam->KeepGoal.dis);
        // printf("m: %d ba: %f bp: %d out_x: %f out_y: %f obs_x: %f obs_y: %f\r\n", _mode, _ball_angle, ball_pos, out_x, out_y, observed_x, observed_y);
        atc->setMode(4);
        atc->setGoStraightXY(out_x, out_y);
    }
}