/*
 *  keeper.hpp
 *
 *  Created on: 2024/3/14
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef APP_ALGO_KEEPER_HPP_
#define APP_ALGO_KEEPER_HPP_

#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>

#include <Lib/pid.hpp>

#include <Module/AttitudeController.hpp>
#include <Module/BatteryVoltageChecker.hpp>
#include <Module/Camera.hpp>
#include <Module/KickerController.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/UI.hpp>

class Keeper {
   public:
    Keeper(MAL* _mcu, AttitudeController* _atc, camera* _cam, KickerController* _kicker, LineSensor* _line, MPU6500* _imu, UI* _ui);
    void init();
    void update();

   private:
    MAL* mcu;
    AttitudeController* atc;
    camera* cam;
    KickerController* kicker;
    LineSensor* line;
    MPU6500* imu;
    UI* ui;

    PID<float> PID_ReturnGoal_X;
    PID<float> PID_ReturnGoal_Y;

    PID<float> PID_LineBack;
    PID<float> PID_GuardGoal;
    PID<float> PID_traceBallY;

    void _returnGoal();
    void _setLinecenter();
    void _guardGoal();

    const float _goal_target = 85;
    const float _ball_distance_target = 50;

    int _mode = 1;
    int _guard_goal_mode = 0;

    int _prev_ball_angle = 0;
    int _prev_ball_distance = 0;
    unsigned int _ball_move_check_time = 0;
};

#endif /* APP_ALGO_KEEPER_HPP_ */