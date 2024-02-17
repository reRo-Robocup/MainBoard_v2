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

// struct camera_object ball;
// struct camera_object at_goal;
// struct camera_object df_goal;


camera::camera(MAL* mcu) {
    _mcu = mcu;
}

void camera::init() {
    AttackColor = YELLOW;
    printf("Camera init\n");
}

void camera::updateFPS() {
    // this->_read_via_buffer();
    // printf("%u\n", _mcu->uartGetChar(MAL::Peripheral_UART::Cam));
}

void camera::_read_by_header() {
    /*  - Dynamixel Protocol 2.0
        | header[4] | BallAngle(16bit) | YellowAngle(16bit) | BlueAngle(16bit)|
        | BallDis(8bit) | enable(8bit) |
    */
    const uint8_t header[4] = {0xFF, 0xFF, 0xFD, 0x00};
    uint8_t errcnt = 0;

    // ヘッダー 受信チェック
    for(int i = 0; i < 4; i++) {
        if(_mcu->uartGetChar(CAM) != header[i])
            errcnt++;
    }

    if(errcnt == 0) {
        // 角度 格納
        for(int i = 0; i < 2; i++) {
            uint8_t _Hdata, _Ldata;
            _Hdata = _mcu->uartGetChar(CAM);
            _Ldata = _mcu->uartGetChar(CAM);
            angle[i] = (_Hdata << 8) | _Ldata;
        }

        // 距離データ 格納
        for(int i = 0; i < 2; i++) {
            distance[i] = _mcu->uartGetChar(CAM);
        }

        // 検出できたか 格納
        uint8_t _data = _mcu->uartGetChar(CAM);
        enable[0] = (bool)_data & 0b00000001;
        enable[1] = (bool)_data & 0b00000010;
        enable[2] = (bool)_data & 0b00000100;
    }
}

void camera::_read_via_buffer() {
    const uint8_t data_size = 10;
    uint8_t data[data_size];
    _mcu->uartReadViaBuffer(CAM, (uint8_t*) data, data_size);

    const uint8_t header[4] = {0xFF, 0xFF, 0xFD, 0x00};
    uint8_t errcnt = 0;

    for(int i = 0; i < 4; i++) {
        if(data[i] != header[i])
            errcnt++;
    }

    if(errcnt == 0) {
        for(int i = 0; i < 3; i++) {
            this->angle[i]    = ((data[4+i] << 8 )| data[5+i]);
            this->distance[i] = data[10+i];
        }
    }
}