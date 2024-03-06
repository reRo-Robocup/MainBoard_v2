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

    logic_main();
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

void logic_main(void) {
    
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

    while (1) {
    }
}