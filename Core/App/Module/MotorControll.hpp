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
    void turn(bool cw);
    void carryBall(int16_t TargetAngle, uint8_t GoalDistance, int16_t IMU_yaw);

   private:
    MAL* _mcu;
    float _duty_to_LAPduty(float duty);
    float _getBatteryVoltage();
};

#endif /* _APP_MODULE_MOTORCONTROLL_HPP_ */