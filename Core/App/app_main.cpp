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
LineSensor line(&mcu);
MPU6500 imu(&mcu);
AttitudeController atc(&mcu, &imu);
UI ui(&mcu);

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
    cam.updateFPS();
    line.update();
    imu.update();
    atc.update();
    //  printf("app_update\n\r");
}

// extern "C" {
void app_main() {
    printf("app_start\n\r");

    app_init();
    while (!imu.isCalibrationed) {
    }
    printf("IMU is Calibrated\n\r");

    while (1) {
        // atc.setMode(2);
        // atc.setTurnAngle(180);
        // mcu.delay_ms(1000);
        // atc.setTurnAngle(90);
        // mcu.delay_ms(1000);

        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor1, 0.75);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor2, 0.75);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor3, 0.75);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor4, 0.75);
        // mcu.delay_ms(1000);
        // atc.setMode(1);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor1, 0.5);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor2, 0.5);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor3, 0.5);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor4, 0.5);
        // mcu.delay_ms(1000);
        //  motor.run(0);

        // if(imu.Yaw < 0) {
        //     motor.roll(0.2, 0.2, 0.2, 0.2);
        // }

        // else {
        //     motor.roll(-0.2,-0.2,-0.2,-0.2);
        // }

        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor1, 0.8);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor2, 0.8);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor3, 0.8);
        // mcu.pwmSetDuty(MAL::Peripheral_PWM::Motor4, 0.8);

        // motor.run(0);

        // ui.Lchika();
        // printf("app_main\n\r");
        printf("Line: %f ", line.angle);
        for (int i = 0; i < 16; i++) {
            printf("%d", line.sensorValue[i] > 600 ? 1 : 0);
        }
        printf(" ");
        for (int i = 16; i < 32; i++) {
            printf("%d", line.sensorValue[i] > 600 ? 1 : 0);
        }
        printf("\n\r");
        // mcu.delay_ms(100);
        // mcu.gpioSetValue(MAL::Peripheral_GPIO::Debug_LED0, 1);
        // printf("rGz: %.4d Gz: %.4f Yaw: %.4f\n\r", imu.raw_Gz, imu.Gz, imu.Yaw);
        // printf("Yaw : %.4f\n\r", imu.Yaw);
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
// }