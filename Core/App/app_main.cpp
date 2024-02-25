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

void app_init();
void app_main();
void app_update();

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
}

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
    atc.setMode(2);

    while (1) {
        atc.setTurnAngle(180);
        printf("%u\n", cam.ball.angle);
    }
}