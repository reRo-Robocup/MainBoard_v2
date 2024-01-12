/*
 *  app_main.cpp
 *
 *  Created on: Dec 7, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#include <app_main.h>

#include <GlobalDefines.h>

#include <Module/Camera.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/UI.hpp>
#include <Module/MotorControll.hpp>

#include <McuAbstractionLayer/stm32f446AbstractionLayer.hpp>

stm32f446AbstractionLayer* mcu;

camera cam(mcu);
LineSensor line(mcu);
MPU6500 imu(mcu);
MotorControll motor(mcu);

void app_init() {
    cam.init();
    line.init();
    imu.init();
    motor.init();
}

void app_main() {

    app_init();

    uint16_t BallAngle = 0;
    uint8_t isBallFront = 5;
    uint16_t angle = 0;

    while (1) {

        // 向き直し
        while(abs(imu.yaw) > 5) {
            bool dir = signbit(imu.yaw);
            motor.turn(dir);
        }

        // ラインセンサー処理
        while(line.isonLine) {
            motor.run(line.angle);
        }

        // キャッチした場合
        while(mcu->adcGetValue(MAL::Peripheral_ADC::BallCatchA)) {
            int16_t GoalAngle = cam.angle[cam.AttackColor];
            if(abs(GoalAngle) > 15) {
                // 正面
                motor.run(GoalAngle);
            }
            else if (abs(GoalAngle > 45)) {
                // 狙える
                motor.carryBall(GoalAngle, cam.distance[cam.AttackColor], imu.yaw);
            }
            else {
                // 狙えない
                unsigned long tim = mcu->millis();
                while(((mcu->millis() - tim) < 2000) && (abs(cam.angle[cam.AttackColor])) > 45) {
                    motor.run(180);
                }
            }
        }

        // 回り込み
        BallAngle = cam.angle[2];
        if(abs(BallAngle) < isBallFront) {
            angle = isBallFront;
        }
        else {
            angle = BallAngle + 30 * ((BallAngle)? -1: 1);
        }
        motor.run(angle);
    }
}
