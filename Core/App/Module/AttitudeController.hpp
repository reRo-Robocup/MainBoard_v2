
/*
 * AttitudeController.hpp
 *
 *  Created on: Feb 15, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_ATTITUDECONTROLLER_HPP_
#define _APP_MODULE_ATTITUDECONTROLLER_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class AttitudeController {
   public:
    AttitudeController(baseMcuAbstractionLayer* mcu);
    void init();
    void update();

    void setMode(int mode);

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
    int _mode;

    void _setPWM(TractionMotors motor, float duty);
};

#endif /* _APP_MODULE_ATTITUDECONTROLLER_HPP_ */