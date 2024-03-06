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
#include <Lib/pid.hpp>
#include <Module/MPU6500.hpp>

class MotorControll {
   public:

    MotorControll(MAL* mcu, MPU6500* imu);

    void init();
    void run(int16_t angle);
    void roll(uint8_t ch, float duty);
    void turn(int16_t targetAngle, int16_t yaw);

    bool isDRVsleep();
    float speed;

   private:
    MAL* _mcu;
    MPU6500* _imu;
    PID<float> _turn_angle_pid;
    
    int16_t map2_180(int16_t angle);
    float _getBatteryVoltage();
    void _write_pwm(uint8_t pin, float duty);
};

#endif /* _APP_MODULE_MOTORCONTROLL_HPP_ */