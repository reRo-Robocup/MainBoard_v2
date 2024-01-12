/*
 * MotorControll.hpp
 *
 *  Created on: Dec 23, 2023
 */

#ifndef APP_HARDWARECONTROLLER_MotorControll_HPP_
#define APP_HARDWARECONTROLLER_MotorControll_HPP_

#include <baseMcuAbstractionLayer.hpp>

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

#endif /* APP_HARDWARECONTROLLER_MotorControll_HPP_ */