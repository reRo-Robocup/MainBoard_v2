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

    int mode = 0;
    atc.setMode(2);

    float angle = 180;

    while (1) {
        atc.setTurnAngle(angle);
        if (mode == 0) {
            angle -= 0.1;
            if (angle < 90) {
                mode = 1;
            }
        } else if (mode == 1) {
            angle += 0.1;
            if (angle > 180) {
                mode = 0;
            }
        }
        printf("batt: %f\r\n", bvc.getVoltage());
        // printf("angle: %f\r\n", angle);
        // printf("%d\r\n", cam.data.ball_angle);
    }
}