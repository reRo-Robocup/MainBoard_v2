/*
 *  Camera.cpp
 *
 *  Created on: Jan 1, 2024
 * 
 *  Author: User, G4T1PR0
 */

#include <Camera.hpp>

#define YELLOW 0
#define BLUE 1

camera::camera(MAL* mcu) {
    _mcu = mcu;
}

void camera::init() {
    AttackColor = YELLOW;
}

void camera::updateFPS() {
    camera::_read_by_header();
}

void camera::_read_by_header() {
    uint8_t header = 0xFF;
    uint8_t _data = _mcu->uartGetChar(MAL::Peripheral_UART::Cam);
    if(_data == header) {
        // 角度 格納
        for(int i = 0; i < 2; i++) {
            uint8_t _Hdata, _Ldata;
            _Hdata = _mcu->uartGetChar(MAL::Peripheral_UART::Cam);
            _Ldata = _mcu->uartGetChar(MAL::Peripheral_UART::Cam);
            angle[i] = ((_Hdata << 8) & 0x0000FF00) | ((_Ldata << 0) & 0x000000FF);
        }
        // 距離データ 格納
        for(int i = 0; i < 2; i++) {
            distance[i] = _mcu->uartGetChar(MAL::Peripheral_UART::Cam);
        }
        // 検出できたか 格納
        isDisable[0] = (_mcu->uartGetChar(MAL::Peripheral_UART::Cam) & 0x10000000) << 8;
        isDisable[1] = (_mcu->uartGetChar(MAL::Peripheral_UART::Cam) & 0x01000000) << 8;
        isDisable[2] = (_mcu->uartGetChar(MAL::Peripheral_UART::Cam) & 0x00100000) << 8;
    }
}