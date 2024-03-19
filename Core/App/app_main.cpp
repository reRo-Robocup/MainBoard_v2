/*
 *  app_main.cpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#include <app_main.h>
#include <Algo/attacker.hpp>
#include <Algo/keeper.hpp>
#include <Lib/pid.hpp>
#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>
#include <Module/AttitudeController.hpp>
#include <Module/BatteryVoltageChecker.hpp>
#include <Module/Camera.hpp>
#include <Module/KickerController.hpp>
#include <Module/MotorControll.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/UI.hpp>
#include <Module/AttitudeController.hpp>

stm32halAbstractionLayer mcu;
BatteryVoltageChecker bvc(&mcu);
KickerController kicker(&mcu);
camera cam(&mcu);
MPU6500 imu(&mcu);
AttitudeController atc(&mcu, &imu);
UI ui(&mcu);
LineSensor line(&mcu, &ui);

Attacker attacker(&mcu, &atc, &cam, &kicker, &line, &imu, &ui);
Keeper keeper(&mcu, &atc, &cam, &kicker, &line, &imu, &ui);

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
        ui.buzzer(300, 10000);
    } else {
        printf("CriticalClear\r\n");
        ui.buzzer(300, 1000);
    }
}

void app_init() {
    mcu.init();
    bvc.init();
    kicker.init();
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

    attacker.init();
    keeper.init();
}

void app_update() {
    bvc.update();
    cam.update();
    kicker.update();
    line.update();
    imu.update();
    atc.update();
    ui.update();

    logic_main();
}

float dis;
uint8_t qty;

PID <float> PID_LineTrace;
float p;
int16_t line_ang;
int16_t toMove;
int8_t dir;

int16_t y;

void logic_main(void) {

    y = imu.Yaw;

    atc.setMode(3);
    atc.setGoStraightAngle(0);
    atc.setGoStraightPower(0.3);


    // line_ang = line.angle;
    // dis = line.getSensDistance();

    // if(line.isonLine && (dis < 80)) {

    //     // p = PID_LineTrace.update(120, dis) / 120;

    //     dir = abs(line.angle) < 110;

    //     // if(abs(p) > 0.5) p = 0.5;

    //     atc.setMode(3);
    //     atc.setGoStraightPower(0.3);


    //     if(dir) {
    //         toMove = 180;
    //     }
    //     else {
    //         toMove = 0;
    //     }

    //     atc.setGoStraightAngle(toMove);
    //     // ui.buzzer(1000,1);
    // }

    // else {
    //     p = 0;
    //     atc.setMode(0);
    // }


    if(ui.getSW()) {
        kicker.setMode(0);
        switch (ui.getRotarySW()){
            case 0:
                mcu.systemReset();
                break;
            case 1:
                kicker.setMode(2);
                break;
            case 2:
                kicker.setMode(3);
                break;
            default:
                break;
        }
    }
}

void app_main() {
    printf("app_start\r\n");
    app_init();

    uint32_t calibration_start_time = 0;
    ui.buzzer(1000, 1000);
    calibration_start_time = mcu.millis();

    while (!imu.isCalibrationed) {
        if ((mcu.millis() - calibration_start_time) > 4000) {
            printf("retry IMU Initialize\r\n");
            imu.init();
            calibration_start_time = mcu.millis();
        }
    }

    printf("IMU is Calibrated\r\n");

    ui.buzzer(2000, 50);
    mcu.delay_ms(100);
    ui.buzzer(1000, 50);

    cam.AttackColor = YELLOW;

    PID_LineTrace.setProcessTime(0.001);
    PID_LineTrace.setPID(0.1,0,0);

    while (1) {
        /* main loop */
    }
}