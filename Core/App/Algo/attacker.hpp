/*
 *  attacker.hpp
 *
 *  Created on: 2024/3/14
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef APP_ALGO_ATTACKER_HPP_
#define APP_ALGO_ATTACKER_HPP_

#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>

#include <Module/AttitudeController.hpp>
#include <Module/BatteryVoltageChecker.hpp>
#include <Module/Camera.hpp>
#include <Module/KickerController.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/UI.hpp>

class Attacker {
   public:
    Attacker(MAL* _mcu, AttitudeController* _atc, camera* _cam, KickerController* _kicker, LineSensor* _line, MPU6500* _imu, UI* _ui);
    void init();
    void update();
    void update2();

   private:
    MAL* mcu;
    AttitudeController* atc;
    camera* cam;
    KickerController* kicker;
    LineSensor* line;
    MPU6500* imu;
    UI* ui;

    PID<float> PID_traceBallX;
    PID<float> PID_traceBallY;

    PID<float> PID_traceBallX2;
    PID<float> PID_traceBallY2;

    const float _ball_distance_target = 45;

    int _mode = 0;
};

#endif /* APP_ALGO_ATTACKER_HPP_ */