/*
 *  app_main.cpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#include <GlobalDefines.h>
#include <app_main.h>
#include <McuAbstractionLayer/stm32f446AbstractionLayer.hpp>
#include <Module/AttitudeController.hpp>
#include <Module/Camera.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/MotorControll.hpp>
#include <Module/UI.hpp>

stm32f446AbstractionLayer mcu;
camera cam(&mcu);
MPU6500 imu(&mcu);
AttitudeController atc(&mcu, &imu);
UI ui(&mcu);
LineSensor line(&mcu, &ui);

// extern "C" {
void app_init();
void app_main();
void app_update();
// };

void app_init() {
    mcu.init();
    cam.init();
    line.init();
    imu.init();
    atc.init();
    ui.init();
    mcu.interruptSetCallback(MAL::Peripheral_Interrupt::T1ms, &app_update);
}

void app_update() {
    cam.update();
    line.update();
    imu.update();
    atc.update();
    ui.update();
    //  printf("app_update\n\r");
}

// extern "C" {
void app_main() {
    printf("app_start\n\r");

    app_init();
    ui.buzzer(1000, 1000);
    while (!imu.isCalibrationed) {
    }
    printf("IMU is Calibrated\n\r");

    ui.buzzer(2000, 50);
    mcu.delay_ms(100);
    ui.buzzer(1000, 50);

    int mode = 0;

    while (1) {
        switch (mode) {
            case 0:
                atc.setMode(3);
                atc.setGoStraightAngle(0);
                atc.setTurnAngle(10);
                atc.setGoStraightPower(0.1);
                // printf("ball_angle: %d\n\r", cam.angle);

                break;

            case 1:
                break;

            default:
                break;
        }

        // if(!isSleep) {
        //     // 向き直し
        //     while (abs(imu.yaw) > 5) {
        //         bool dir = signbit(imu.yaw);
        //         // motor.turn(dir);
        //     }

        //     // ラインセンサー処理
        //     while (line.isonLine) {
        //         motor.run(line.angle);
        //     }

        //     // キャッチした場合
        //     while (mcu.adcGetValue(MAL::Peripheral_ADC::BallCatchA) > BallCatchThreshold[0]) {
        //         int16_t GoalAngle = cam.angle[cam.AttackColor];
        //         // 正面
        //         if (abs(GoalAngle) > 15) {
        //             motor.run(GoalAngle);
        //         }
        //         // 狙える
        //         else if (abs(GoalAngle > 45)) {
        //             motor.carryBall(GoalAngle, cam.distance[cam.AttackColor], imu.yaw);
        //         }
        //         // 狙えない
        //         else {
        //             unsigned long tim = mcu.millis();
        //             while (((mcu.millis() - tim) < 2000) && (abs(cam.angle[cam.AttackColor])) > 45) {
        //                 motor.run(180);
        //             }
        //         }
        //     }

        //     // 回り込み
        //     BallAngle = cam.angle[2];
        //     if (abs(BallAngle) < isBallFront) {
        //         angle = isBallFront;
        //     } else {
        //         angle = BallAngle + 30 * ((BallAngle) ? -1 : 1);
        //     }
        //     motor.run(angle);
        // }
    }
}