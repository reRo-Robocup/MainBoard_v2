/*
 *  app_main.cpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#include <GlobalDefines.h>
#include <app_main.h>
#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>
#include <Module/AttitudeController.hpp>
#include <Module/BatteryVoltageChecker.hpp>
#include <Module/Camera.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/MotorControll.hpp>
#include <Module/UI.hpp>

stm32halAbstractionLayer mcu;
BatteryVoltageChecker bvc(&mcu);
camera cam(&mcu);
MPU6500 imu(&mcu);
AttitudeController atc(&mcu, &imu);
UI ui(&mcu);
LineSensor line(&mcu, &ui);

void app_init();
void app_main();
void app_update();

void BatteryVoltageWarning(bool flag) {
    if (flag) {
        printf("BatteryVoltageWarning\r\n");
        ui.buzzer(500, 10000);
    } else {
        printf("WarningClear\r\n");
        ui.buzzer(500, 1000);
    }
}

void BatteryVoltageCritical(bool flag) {
    if (flag) {
        printf("BatteryVoltageCritical\r\n");
        ui.buzzer(1000, 10000);
    } else {
        printf("CriticalClear\r\n");
        ui.buzzer(1000, 1000);
    }
}

void app_init() {
    mcu.init();
    bvc.init();
    cam.init();
    line.init();
    imu.init();
    atc.init();
    ui.init();
    mcu.interruptSetCallback(MAL::Peripheral_Interrupt::T1ms, &app_update);

    bvc.setWarningVoltage(10);
    bvc.setWarningTime(1);
    bvc.setWarningCallback(&BatteryVoltageWarning);

    bvc.setCriticalVoltage(8);
    bvc.setCriticalTime(1);
    bvc.setCriticalCallback(&BatteryVoltageCritical);
}

void app_update() {
    bvc.update();
    cam.update();
    line.update();
    imu.update();
    atc.update();
    ui.update();
}

float map (float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MoveOnlyX(int16_t ObjAngle, int16_t TargetAngle) {

    int16_t angle = ObjAngle;
    int16_t diff = TargetAngle - angle;

    int8_t dir;

    if(diff != 0) {
        dir = (diff / abs(diff));
    }

    // printf("%d\n", diff);

    if(abs(diff) < 5) {
        printf("STop\n");
        atc.setMode(0);
    }

    else {
        atc.setMode(3);
        printf("Idou\n");
        atc.setGoStraightAngle(-90 * dir);
    }
}

void app_main() {
    uint32_t calibration_start_time = 0;
    printf("app_start\r\n");

    app_init();
    ui.buzzer(1000, 1000);
    calibration_start_time = mcu.millis();
    while (!imu.isCalibrationed) {
        if (mcu.millis() - calibration_start_time > 2000) {
            printf("retry IMU Initialize\r\n");
            imu.init();
            calibration_start_time = mcu.millis();
        }
    }
    printf("IMU is Calibrated\r\n");

    ui.buzzer(2000, 50);
    mcu.delay_ms(100);
    ui.buzzer(1000, 50);
    atc.setGoStraightPower(0.8);

    while (1) {

        // atc.setMode(0);

        // printf("%d\n", cam.data.ball_angle);
        // int16_t ballAngle = cam.data.ball_angle;
        // if(ballAngle > 180) ballAngle -= 360;
        // // printf("%d\n", ballAngle);

        MoveOnlyX(cam.data.blue_angle, 180);

        // atc.setGoStraightAngle(-90);

        // float dt = 0;
        // for(int i = 0; i < 100; i++) {
        //     dt += 0.01;
        //     atc.setGoStraightPower(dt);
        //     atc.setGoStraightAngle(180);
        // }

        // int16_t MyGoal_Angle = cam.data.blue_angle;
        // printf("%d\n", MyGoal_Angle);

        // xベクトル
        // float goal_Vx = cos(MyGoal_Angle * (M_PI / 180));
        // printf("BlueAngle: %d Goal_Vx: %f \n", MyGoal_Angle, goal_Vx);

        // // yベクトル
        // uint8_t goal_distance = cam.data.blue_distance;

        // if(0) {
        //     // ボールが見つからない時, 自陣復帰
        //     atc.setMode(0);
        //     // if((cam.data.blue_distance > 10) && (abs(cam.data.ball_angle) < 5)) {
        //     //     atc.setGoStraightAngle(cam.data.ball_angle);
        //     // }

        // }
        // else {
        //     atc.setMode(3);
        //     atc.setGoStraightPower(0.6);
        //     int16_t angle = cam.data.ball_angle;
        //     int16_t toMove = 0;

        //     if(angle > 180) angle -= 360;
        //     if(angle < -180) angle += 360;
        //     // printf("%d\n", angle);

        //     if((angle < 15) && (angle > -15)) {
        //         toMove = angle;
        //     }
        //     else {
        //         bool dir = (angle > 0);
        //         if(dir) toMove = angle + 30;
        //         else toMove = angle - 30;
        //         // atc.setGoStraightAngle(angle + (dir * 30));
        //     }

        //     atc.setGoStraightAngle(toMove);
        // }
    }
}