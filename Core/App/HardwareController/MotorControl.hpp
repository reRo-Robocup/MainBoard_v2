/*
 * lineSensorAlgorithm.hpp
 *
 *  Created on: Dec 23, 2023
 */

#ifndef APP_HARDWARECONTROLLER_MOTORCONTROL_HPP_
#define APP_HARDWARECONTROLLER_MOTORCONTROL_HPP_

class lineSensorAlgorithm {
   public:
    MotorControl(Devices* devices);
    void init();
    void run(uint8_t angle, uint8_t speed);

   private:
    Devices* _devices;
};

#endif /* APP_HARDWARECONTROLLER_MOTORCONTROL_HPP_ */