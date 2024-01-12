/*
 *  app_main.cpp
 *
 *  Created on: Dec 7, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#include <app_main.h>

#include <GlobalDefines.h>

#include <Module/Camera.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/UI.hpp>
#include <Module/MotorControll.hpp>

void app_init() {

}

void app_main() {
    app_init();

    // const int isBallFront = 30;
    // int16_t angle;
    // uint16_t speed;

    // uint16_t Ball_threshold = 3078;

    while (1) {

        // // スピード決定 実装中
        // speed = devices.mcu->adcGetValue(MAL::Peripheral_ADC::BatteryVoltage) / BATT_MIN_V;

        // // 向き直し
        // while(abs(devices.mpu6500->yaw) > 5) {
        //     bool dir = signbit(devices.mpu6500->yaw);
        //     hwc.motor->turn(dir, speed);
        // }

        // // ラインセンサー処理
        // while(hwc.lineSensorAlgo->isOnLine) {
        //     hwc.motor->run(hwc.lineSensorAlgo->angle, speed);
        // }

        // // キャッチした場合
        // while(devices.mcu->adcGetValue(MAL::Peripheral_ADC::BallCatchA) > Ball_threshold) {
        //     int16_t GoalAngle = devices.camera->angles[devices.camera->AttackColor];
        //     if(abs(GoalAngle) > 15) {
        //         // 正面
        //         hwc.motor->run(GoalAngle, speed);
        //     }
        //     else if (abs(GoalAngle > 45)) {
        //         // 狙える
        //         hwc.motor->carryBall(GoalAngle, devices.camera->goalDistance[devices.camera->AttackColor], speed, devices.mpu6500->yaw);
        //     }
        //     else {
        //         // 狙えない
        //         unsigned long tim = devices.mcu->millis();
        //         while(((devices.mcu->millis() - tim) < 2000) && (abs(devices.camera->angles[devices.camera->AttackColor])) > 45) {
        //             hwc.motor->run(180, speed / 2);
        //         }
        //     }
        // }

        // // 回り込み
        // BallAngle = devices.camera->angles[2];

        // if(abs(BallAngle) < isBallFront) {
        //     angle = isBallFront;
        // }
        // else {
        //     angle = BallAngle + 30 * ((BallAngle)? -1: 1);
        // }

        // hwc.motor->run(angle, speed);
    }
}
