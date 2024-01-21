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
}

void camera::updateFPS() {
    this->_read_by_header();
}

void camera::_read_by_header() {
    /*  - Dynamixel Protocol 2.0
        | header[4] | BallAngle(16bit) | YellowAngle(16bit) | BlueAngle(16bit)|
        | BallDis(8bit) | enable(8bit) |
    */
    const uint8_t header[4] = {0xFF, 0xFF, 0xFD, 0x00};
    uint8_t _header_errcnt = 0;

    // ヘッダー 受信チェック
    for(int i = 0; i < 4; i++) {
        if(_mcu->uartGetChar(CAM) != header[i])
            _header_errcnt++;
    }

    if(_header_errcnt == 0) {
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
        enable[0] = (_mcu->uartGetChar(CAM) & 0x10000000) >> 8;
        enable[1] = (_mcu->uartGetChar(CAM) & 0x01000000) >> 8;
        enable[2] = (_mcu->uartGetChar(CAM) & 0x00100000) >> 8;
    }
}