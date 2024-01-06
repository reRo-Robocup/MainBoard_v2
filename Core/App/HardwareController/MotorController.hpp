/*
 * MotorController.hpp
 *
 *  Created on: Dec 23, 2023
 */

#ifndef APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_

class MotorController {
   public:
    MotorController(Devices* devices);
    void init();
    void run(uint8_t angle, uint8_t speed);
    void MotorRoll(int motor, float duty);
    void turn(bool cw, uint8_t speed);
    void carryBall(int16_t TargetAngle, uint8_t GoalDistance, uint8_t speed, int16_t IMU_yaw);

   private:
    Devices* _devices;
    float _duty_to_LAPduty(float duty);
};

#endif /* APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_ */