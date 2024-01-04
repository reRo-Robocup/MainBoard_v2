/*
 * app_main.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <app_main.h>
#include <Devices/Devices.hpp>
#include <HardwareController/HardwareController.hpp>

#include "./GlobalDefines.h"

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
    int8_t Ball_dir = 0;

    while (1) {
        devices.update();
        hwc.update();

        Ball_dir = BallAngle / abs(BallAngle);

        // ラインセンサー処理
        while(hwc.lineSensorAlgo->isOnLine) {
            hwc.motor->run(hwc.lineSensorAlgo->angle, 100);
        }

        // 回り込み
        if(abs(BallAngle) < isBallFront) {
            angle = isBallFront;
        }
        else {
            angle = BallAngle + 30;
        }

        // IMU処理


        // BallAngle = hwc.camera->BallAngle;
        // debug_angle = hwc.lineSensorAlgo->angle;
        // for (int i = 0; i < 32; i++) {
        //     debug_sensor[i] = devices.lineSensor->sensorValue[i];
        // }
    }
}
