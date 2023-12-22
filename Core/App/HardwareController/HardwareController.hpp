/*
 * HardwareController.hpp
 *
 *  Created on: Dec 8, 2023
 */

#ifndef APP_HARDWARECONTROLLER_HARDWARECONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_HARDWARECONTROLLER_HPP_

#include <Devices/Devices.hpp>
#include <HardwareController/lineSensorAlgorithm.hpp>
class HardwareController {
   public:
    HardwareController(Devices* devices);
    void init();
    void update();

    lineSensorAlgorithm* lineSensorAlgo;

   private:
    Devices* _devices;
};

#endif /* APP_HARDWARECONTROLLER_HARDWARECONTROLLER_HPP_ */