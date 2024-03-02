/*
 *  RaspberryPiCommunicationController.hpp
 *
 *  Created on: Mar 1, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_RASPBERRYPI_COMMUNICATION_CONTROLLER_HPP_
#define _APP_MODULE_RASPBERRYPI_COMMUNICATION_CONTROLLER_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>
#include <Module/MPU6500.hpp>

class RaspberryPiCommunicationController {
   public:
    RaspberryPiCommunicationController(baseMcuAbstractionLayer* mcu, MPU6500* imu);

    struct RxData {
        uint16_t a;
    };

    RxData data;

    void update();

   private:
    baseMcuAbstractionLayer* _mcu;
    MPU6500* _imu;

    union TxData {
        uint8_t tx_buffer[2];
        uint16_t yaw_angle;
    };

    union RxDataParser {
        uint8_t rx_buffer[sizeof(RxData)];
        RxData parsed_data;
    };

    TxData tx_data;
    RxDataParser rx_data_parser;

    uint8_t _header[4] = {0xFF, 0xFF, 0xFD, 0x00};

    int _rx_mode = 0;
    unsigned int _rx_data_index;

    void _send_data();
    void _parse_data();
};

#endif /* _APP_MODULE_RASPBERRYPI_COMMUNICATION_CONTROLLER_HPP_ */