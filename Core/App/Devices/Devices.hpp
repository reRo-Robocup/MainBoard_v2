/*
 * Devices.hpp
 *
 *  Created on: Dec 7, 2023
 */

#ifndef APP_DEVICES_Devices_H_
#define APP_DEVICES_Devices_H_

#include <Devices/Driver/lineSensorDriver.hpp>
#include <Devices/McuAbstractionLayer/stm32f446AbstractionLayer.hpp>

class Devices {
   public:
    Devices();
    void init();
    void update();

    stm32f446AbstractionLayer* mcu;
    lineSensorDriver* lineSensor;
};

#endif /* APP_DEVICES_Devices_H_ */
