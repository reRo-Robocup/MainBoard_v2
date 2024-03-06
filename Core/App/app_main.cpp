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
#include <Lib/pid.hpp>

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
void logic_main();

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
    bvc.setWarningTime(100);
    bvc.setWarningCallback(&BatteryVoltageWarning);

    bvc.setCriticalVoltage(8);
    bvc.setCriticalTime(100);
    bvc.setCriticalCallback(&BatteryVoltageCritical);
}

void app_update() {
    bvc.update();
    cam.update();
    line.update();
    imu.update();
    atc.update();
    ui.update();

    logic_main();
}

void MoveOnlyX(int16_t ObjAngle, int16_t TargetAngle) {
    int16_t angle = ObjAngle;
    int16_t diff = TargetAngle - angle;

    int8_t dir;

    if (diff != 0) {
        dir = (diff / abs(diff));
    }

    // printf("%d\n", diff);

    if (abs(diff) < 5) {
        printf("STop\n");
        atc.setMode(0);
    }

    else {
        atc.setMode(3);
        printf("Idou\n");
        atc.setGoStraightAngle(-90 * dir);
    }
}


PID<float> pid_goal_x;
PID<float> pid_goal_y;

float mv_xVector, mv_yVector;
float _speed_x, _speed_y;

void ReturnMyGoal(bool isBlue) {
    atc.setMode(3);

    int16_t angle = (isBlue)? cam.data.blue_angle: cam.data.yellow_angle;
    angle = 90 - cam.data.blue_angle;
    mv_xVector = cos(angle * deg_to_rad);
    _speed_x = pid_goal_x.update(0, mv_xVector);

    uint8_t distance = (isBlue)? cam.data.blue_distance: cam.data.yellow_distance;
    uint8_t goal_dis_threshold = 80;
    _speed_y = pid_goal_y.update(goal_dis_threshold, distance) * -1;
    if(abs(_speed_y) > 20) _speed_y = 20;
    _speed_y /= 20;

    atc.setGoStraightPower(abs(_speed_y));
    int8_t dir = signbit(_speed_y)? -1: 1;
    if(dir) atc.setGoStraightAngle(0);
    else atc.setGoStraightAngle(180);
}

void logic_main(void) {
    if(line.isonLine) {
        atc.setGoStraightAngle(line.angle);
    }
    else {
        ReturnMyGoal(!cam.AttackColor);
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

    pid_goal_x.setPID(0.9,0,0.4);
    pid_goal_x.setProcessTime(0.001);

    pid_goal_y.setPID(0.9,0,0.4);
    pid_goal_y.setProcessTime(0.001);

    cam.AttackColor = YELLOW;

    while (1) {
        // printf("Ba %d Bl %d Yl %d\r\n", cam.data.isBallDetected, cam.data.isBlueDetected, cam.data.isYellowDetected);
    }
}