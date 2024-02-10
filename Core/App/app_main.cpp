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
#include <Module/Camera.hpp>
#include <Module/LineSensor.hpp>
#include <Module/MPU6500.hpp>
#include <Module/MotorControll.hpp>
#include <Module/UI.hpp>

stm32f446AbstractionLayer mcu;
camera cam(&mcu);
LineSensor line(&mcu);
MPU6500 imu(&mcu);
MotorControll motor(&mcu);
UI ui(&mcu);

void app_main();
void app_update();

void app_init() {
    mcu.init();
    cam.init();
    line.init();
    imu.init();
    motor.init();
    ui.init();
    mcu.interruptSetCallback(MAL::Peripheral_Interrupt::T1ms, &app_update);
}

void app_update() {
    // cam.updateFPS();
    // line.update();
    // imu.update();
    // printf("app_update\n");
}

void app_main() {
    printf("app_start\n\r");

    app_init();
    // imu.calibration();

    while (1) {
        // motor.turn(0,0,0);
        // motor.run(0);
<<<<<<< HEAD

        // ui.Lchika();
        // printf("app_main\n\r");
        mcu.delay_ms(100);
        mcu.gpioSetValue(MAL::Peripheral_GPIO::Debug_LED0, 1);
        printf("rGz: %.4d Gz: %.4f Yaw: %.4f\n\r", imu.rGz, imu.Gz, imu.Yaw);
=======
        // mcu.delay_ms(100);

        // ui.Lchika();
        // printf("app_main\n\r");
        // mcu.delay_ms(10);
        // mcu.gpioSetValue(MAL::Peripheral_GPIO::Debug_LED0, 1);
        // printf("rGz: %d Gz: %.4f Yaw: %.4f\n\r", imu.rGz, imu.Gz, imu.Yaw);
>>>>>>> 00626f1 ([update] Main)
        // printf("Ax%.4f Ay:%.4f Az:%.4f Gz%.4f\n\r", imu.Ax, imu.Ay, imu.Az, imu.Yaw);
        //  uint16_t val = imu.zg;
        //  printf("%u\n", val);

        // for(int i = 0; i < 3; i++) {
        //     ui.setLED(i,0);
        //     mcu.delay_ms(200);
        // }

        // for(int i = 0; i < 3; i++) {
        //     ui.setLED(i,1);
        //     mcu.delay_ms(200);
        // }

        // ui.setLED(0,1);
        // ui.setLED(1,1);
        // ui.setLED(2,1);

        // mcu.delay_ms(300);

        // ui.setLED(0,0);
        // ui.setLED(1,0);
        // ui.setLED(2,0);

        // mcu.delay_ms(300);

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
