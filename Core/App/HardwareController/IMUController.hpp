/*
 * IMUController.hpp
 *
 *  Created on: Dec 23, 2023
 */

#ifndef APP_HARDWARECONTROLLER_IMUCONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_IMUCONTROLLER_HPP_

class IMUController {
   public:
    IMUController(Devices* devices);
    void init();
    void update();
   private:
    Devices* _devices;
};

#endif