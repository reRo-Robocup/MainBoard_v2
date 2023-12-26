/*
 * CameraDriver.hpp
 *
 *  Created on: Dec 26, 2023
 */

#ifndef APP_DEVICES_DRIVER_CAMERADRIVER_HPP_
#define APP_DEVICES_DRIVER_CAMERADRIVER_HPP_
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

#define YELLOW 0
#define BLUE 1

class CameraDriver {
   public:
    CameraDriver(MAL* mcu);
    void init();
    void updateFPS();
    bool AttackColor;
    uint16_t angles[3] = {0};
    uint16_t goalDistance[2] = {0};

   private:
    MAL* _mcu;
};

#endif /* APP_DEVICES_DRIVER_CAMERADRIVER_HPP_ */
