
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
    void setGoStraightPower(float power);
    void setTurnAngle(int angle);

    void setGoStraightXY(float x, float y);

    void setTargetPositionXY(float x, float y);

    enum TractionMotors {
        Motor1,
        Motor2,
        Motor3,
        Motor4
    };

    struct MotorData {
        MAL::Peripheral_PWM pin;
        int motorAngle;
        bool isConnectionReversed;
    };

    MotorData motor_data[4];

   private:
    baseMcuAbstractionLayer* _mcu;
    MPU6500* _imu;
    PID<float> _turn_angle_pid;
    PID<float> _position_x_pid;
    PID<float> _position_y_pid;

    int _mode;

    int _go_straight_angle;
    float _go_straight_power = 0.8;

    float _go_straight_x = 0;
    float _go_straight_y = 0;

    float _target_position_x = 0;
    float _target_position_y = 0;

    int _turn_angle;

    void _setPWM(TractionMotors motor, float duty);
};

#endif /* _APP_MODULE_ATTITUDECONTROLLER_HPP_ */