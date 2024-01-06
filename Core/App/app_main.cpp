/*
 * app_main.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <app_main.h>
#include <Devices/Devices.hpp>
#include <HardwareController/HardwareController.hpp>

#include "GlobalDefines.h"

Devices devices;
HardwareController hwc(&devices);

int16_t BallAngle;

void app_init() {
    devices.init();
    hwc.init();
}

void app_main() {
    app_init();

    const int isBallFront = 30;
    int16_t angle;
    uint16_t speed = 255;

    uint16_t Ball_threshold = 3078;

    while (1) {
        devices.update();
        hwc.update();

        // 向き直し
        while(abs(devices.mpu6500->yaw) > 5) {
            bool dir = signbit(devices.mpu6500->yaw);
            hwc.motor->turn(dir, speed);
        }

        // ラインセンサー処理
        while(hwc.lineSensorAlgo->isOnLine) {
            hwc.motor->run(hwc.lineSensorAlgo->angle, 100);
        }

        // キャッチした場合
        while(devices.mcu->adcGetValue(MAL::Peripheral_ADC::BallCatchA) > Ball_threshold) {
            int16_t GoalAngle = devices.camera->angles[devices.camera->AttackColor];
            if(abs(GoalAngle) > 15) {

            }
            else {

            }
            hwc.motor->carryBall(0,0,0);
        }

        // 回り込み
        BallAngle = devices.camera->angles[2];

        if(abs(BallAngle) < isBallFront) {
            angle = isBallFront;
        }
        else {
            angle = BallAngle + 30;
        }

        hwc.motor->run(angle,100);
    }
}
