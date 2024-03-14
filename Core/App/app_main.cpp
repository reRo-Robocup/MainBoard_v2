/*
 *  app_main.cpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#include <GlobalDefines.h>
#include <app_main.h>
#include <Lib/pid.hpp>
#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>
#include <Module/AttitudeController.hpp>
#include <Module/BatteryVoltageChecker.hpp>
#include <Module/Camera.hpp>
#include <Module/KickerController.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/MotorControll.hpp>
#include <Module/UI.hpp>

stm32halAbstractionLayer mcu;
BatteryVoltageChecker bvc(&mcu);
KickerController kicker(&mcu);
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

    if (ui.getSW()) {
        kicker.setMode(0);
        switch (ui.getRotarySW()) {
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

PID<float> pid_ReturnMyGoal;

float returnAngle;
float power;

int16_t MyGoal_Angle;
int16_t distance;

void ReturnMyGoal() {
    atc.setMode(3);

    // MyGoal_Angle = (cam.AttackColor)? cam.data.blue_angle: cam.data.yellow_angle;
    // distance = (cam.AttackColor)? cam.data.blue_distance: cam.data.yellow_distance;
    MyGoal_Angle = cam.data.blue_angle;
    distance = cam.data.blue_distance;

    // MyGoal_Angle = 90 - cam.data.blue_angle;
    // mv_xVector = cos(MyGoal_Angle * deg_to_rad);
    // _speed_x = pid_goal_x.update(0, mv_xVector);

    uint8_t goal_dis_threshold = 85;
    power = pid_ReturnMyGoal.update(goal_dis_threshold, distance);
    if (abs(power) > 20)
        power = 20;
    power /= 20;

    int8_t dir = signbit(power);

    if (dir) {
        atc.setGoStraightAngle(MyGoal_Angle);
        atc.setGoStraightPower(abs(power));
    }

    else {
        atc.setGoStraightAngle(MyGoal_Angle + 180);
        atc.setGoStraightPower(abs(power));
    }

    // returnAngle = (360 - (atan2(_speed_y,_speed_x) * rad_to_deg * -1)) + 180;
    // returnAngle = (360 - (atan2(_speed_y,_speed_x) * rad_to_deg * -1));
    // while(returnAngle > 180) returnAngle -= 360;
    // while(returnAngle < -180)returnAngle += 360;

    // // power = sqrt(pow(_speed_x,2) + pow(_speed_y, 2));
    // power = (_speed_x + _speed_y) / 2;
    // // それぞれabsを取る

    // atc.setGoStraightPower(abs(power));
    // atc.setGoStraightAngle(returnAngle);
    // int8_t dir = signbit(_speed_y);
    // if(dir) atc.setGoStraightAngle(180);
    // else atc.setGoStraightAngle(0);
}

int16_t line_angle;
bool line_isonline;

int16_t line_dis;

void logic_main(void) {
    // line_angle = line.angle;
    // line_isonline = line.isonLine;

    // if(line_isonline) {
    //     atc.setMode(3);
    //     atc.setGoStraightPower(0.8);
    //     atc.setGoStraightAngle(line_angle);
    // }
    // else {
    //     atc.setMode(0);
    //     atc.setGoStraightPower(0);
    // }

    // if(line.isonLine) {
    //     atc.setMode(3);
    //     atc.setGoStraightPower(0.8);
    //     atc.setGoStraightAngle(line_angle);
    // }
    // else {
    ReturnMyGoal();
    // }
}

void app_main() {
    printf("app_start\r\n");
    app_init();

    uint32_t calibration_start_time = 0;
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

    atc.setMode(3);
    kicker.setMode(1);

    pid_ReturnMyGoal.setPID(1.0, 0, 0.4);
    pid_ReturnMyGoal.setProcessTime(0.001);

    cam.AttackColor = YELLOW;

    int mode = 0;
    unsigned long long time = 0;
    // while (1) {
    //     mcu.delay_ms(500);
    //     kicker.setMode(1);
    //     mcu.delay_ms(500);
    //     kicker.setMode(2);
    //     mcu.delay_ms(500);
    //     kicker.setMode(1);
    //     mcu.delay_ms(500);
    //     kicker.setMode(3);
    // }

    while (1) {
        printf("%d\r\n", ui.getRotarySW());
        // ui.getRotarySW();
        // printf("Ba %d Bl %d Yl %d\r\n", cam.data.isBallDetected, cam.data.isBlueDetected, cam.data.isYellowDetected);
    }
}