/*
 * MotorControll.hpp
 *
 *  Created on: Dec 23, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_MOTORCONTROLL_HPP_
#define _APP_MODULE_MOTORCONTROLL_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class MotorControll {
   public:
    MotorControll(MAL* mcu);
    int8_t speed;
    void init();
    void run(uint8_t angle);
    void MotorRoll(int motor, float duty);
    void carryBall(int16_t TargetAngle, uint8_t GoalDistance, int16_t IMU_yaw);
    void approach_Ball(int16_t BallAngle, uint16_t BallDistance, int16_t IMU_yaw, int16_t IMU_Vx, int16_t IMU_Vy);
    void turn(int16_t yaw, int16_t za, int16_t targetAgle);

   private:
    MAL* _mcu;
    float _duty_to_LAPduty(float duty);
    float _getBatteryVoltage();
};

#endif /* _APP_MODULE_MOTORCONTROLL_HPP_ */