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

   private:
    Devices* _devices;
};

#endif /* APP_HARDWARECONTROLLER_MOTORCONTROLLER_HPP_ */