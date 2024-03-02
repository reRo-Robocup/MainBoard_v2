/*
 *  RaspberryPiCommunicationController.cpp
 *
 *  Created on: Mar 1, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "RaspberryPiCommunicationController.hpp"

RaspberryPiCommunicationController::RaspberryPiCommunicationController(baseMcuAbstractionLayer* mcu, MPU6500* imu) {
    _mcu = mcu;
    _imu = imu;
}

void RaspberryPiCommunicationController::update() {
    _send_data();
    _parse_data();
}

void RaspberryPiCommunicationController::_send_data() {
    tx_data.yaw_angle = _imu->Yaw;
    _mcu->uartWriteViaBuffer(MAL::Peripheral_UART::Cam, (uint8_t*)&_header, sizeof(_header));
    _mcu->uartWriteViaBuffer(MAL::Peripheral_UART::Cam, (uint8_t*)&tx_data.tx_buffer, sizeof(TxData));
}

void RaspberryPiCommunicationController::_parse_data() {
    uint8_t data[64] = {0};
    uint32_t data_size = _mcu->uartGetRxDataSize(MAL::Peripheral_UART::Cam);
    _mcu->uartReadViaBuffer(MAL::Peripheral_UART::Cam, data, data_size);

    for (uint32_t i = 0; i < data_size; i++) {
        switch (_rx_mode) {
            case 0:
                if (data[i] == _header[0]) {
                    _rx_mode = 1;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 1:
                if (data[i] == _header[1]) {
                    _rx_mode = 2;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 2:
                if (data[i] == _header[2]) {
                    _rx_mode = 3;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 3:
                if (data[i] == _header[3]) {
                    _rx_mode = 4;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 4:
                rx_data_parser.rx_buffer[_rx_data_index++] = data[i];
                if (_rx_data_index > sizeof(rx_data_parser.rx_buffer)) {
                    _rx_mode = 0;
                    _rx_data_index = 0;
                    this->data = rx_data_parser.parsed_data;
                }
                break;

            default:
                break;
        }
    }
}
