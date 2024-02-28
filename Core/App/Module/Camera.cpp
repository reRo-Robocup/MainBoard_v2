/*
 *  Camera.cpp
 *
 *  Created on: Jan 1, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "Camera.hpp"

#define YELLOW 0
#define BLUE 1

#define CAM MAL::Peripheral_UART::Cam

camera::camera(MAL* mcu) {
    _mcu = mcu;
}

void camera::init() {
    AttackColor = YELLOW;
    printf("Camera init\n");
}

void camera::update() {
    this->_read_via_buffer();
    // printf("%u\n", _mcu->uartGetChar(MAL::Peripheral_UART::Cam));
}

void camera::_read_by_header() {
    /*  - Protocol
        | header[4] | BallAngle(16bit) | YellowAngle(16bit) | BlueAngle(16bit)|
        | BallDis(8bit) | enable(8bit) |
    */
    const uint8_t header[4] = {0xFF, 0xFF, 0xFD, 0x00};
    uint8_t errcnt = 0;

    // ヘッダー 受信チェック
    for (int i = 0; i < 4; i++) {
        if (_mcu->uartGetChar(CAM) != header[i])
            errcnt++;
    }

    // if (errcnt == 0) {
    //     // 角度 格納
    //     for (int i = 0; i < 2; i++) {
    //         uint8_t _Hdata, _Ldata;
    //         _Hdata = _mcu->uartGetChar(CAM);
    //         _Ldata = _mcu->uartGetChar(CAM);
    //         angle[i] = (_Hdata << 8) | _Ldata;
    //     }

    //     // 距離データ 格納
    //     for (int i = 0; i < 2; i++) {
    //         distance[i] = _mcu->uartGetChar(CAM);
    //     }

    //     // 検出できたか 格納
    //     uint8_t _data = _mcu->uartGetChar(CAM);
    //     enable[0] = (bool)_data & 0b00000001;
    //     enable[1] = (bool)_data & 0b00000010;
    //     enable[2] = (bool)_data & 0b00000100;
    // }
}

void camera::_read_via_buffer() {
    uint8_t data[64] = {0};
    uint32_t data_size = _mcu->uartGetRxDataSize(CAM);
    _mcu->uartReadViaBuffer(CAM, data, data_size);

    const uint8_t header[4] = {0xFF, 0xFF, 0xFD, 0x00};

    for (uint32_t i = 0; i < data_size; i++) {
        switch (_rx_mode) {
            case 0:
                if (data[i] == header[0]) {
                    _rx_mode = 1;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 1:
                if (data[i] == header[1]) {
                    _rx_mode = 2;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 2:
                if (data[i] == header[2]) {
                    _rx_mode = 3;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 3:
                if (data[i] == header[3]) {
                    _rx_mode = 4;
                } else {
                    _rx_mode = 0;
                }
                break;

            case 4:
                camera_rx_data_parser.rx_buffer[_rx_data_index++] = data[i];
                if (_rx_data_index > sizeof(camera_rx_data_parser.rx_buffer)) {
                    _rx_mode = 0;
                    _rx_data_index = 0;
                    this->data = camera_rx_data_parser.parsed_data;
                }
                break;

            default:
                break;
        }
    }
}
