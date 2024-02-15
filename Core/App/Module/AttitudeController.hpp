
/*
 * AttitudeController.hpp
 *
 *  Created on: Feb 15, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_ATTITUDECONTROLLER_HPP_
#define _APP_MODULE_ATTITUDECONTROLLER_HPP_

#include <Lib/pid.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>
#include <Module/MPU6500.hpp>

class AttitudeController {
   public:
    AttitudeController(baseMcuAbstractionLayer* mcu, MPU6500* imu);
    void init();
    void update();

    void setMode(int mode);

    void setGoStraightAngle(int16_t angle);
    void setTurnAngle(int angle);

    enum TractionMotors {
        Motor1,
        Motor2,
        Motor3,
        Motor4
    };

    struct MotorData {
        MAL::Peripheral_PWM pin;
        int motorAngle;
        bool isConnectionReserved;
    };

    MotorData motor_data[4];

   private:
    baseMcuAbstractionLayer* _mcu;
    MPU6500* _imu;
    PID _turn_angle_pid;
    int _mode;

    int _go_straight_angle;
    int _turn_angle;

    void _setPWM(TractionMotors motor, float duty);
};

#endif /* _APP_MODULE_ATTITUDECONTROLLER_HPP_ */