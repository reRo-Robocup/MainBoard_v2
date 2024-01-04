/*
 * IMUController.hpp
 *
 *  Created on: Dec 23, 2023
 */

#ifndef APP_HARDWARECONTROLLER_CAMERACONTROLLER_HPP_
#define APP_HARDWARECONTROLLER_CAMERACONTROLLER_HPP_

class CameraController {
   public:
    CameraController(Devices* devices);
    void init();
    void update();
   private:
    Devices* _devices;
};

#endif