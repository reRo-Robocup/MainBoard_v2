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
    void MotorController::MotorRoll(int motor, float duty);

   private:
    Devices* _devices;
};

#endif /* APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_ */