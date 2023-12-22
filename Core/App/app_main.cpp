/*
 * app_main.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <app_main.h>
#include <Devices/Devices.hpp>
#include <HardwareController/HardwareController.hpp>

Devices devices;
HardwareController hwc(&devices);

uint16_t debug_sensor[32] = {0};
bool debug_isOnLine[32] = {0};
float debug_angle;

void app_init() {
    devices.init();
    hwc.init();
}
void app_main() {
    app_init();

    while (1) {
        devices.update();
        hwc.update();
        debug_angle = hwc.lineSensorAlgo->angle;
        for (int i = 0; i < 32; i++) {
            debug_sensor[i] = devices.lineSensor->sensorValue[i];
        }
    }
}
