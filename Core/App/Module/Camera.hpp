/*
 *  Camera.cpp
 *
 *  Created on: Jan 1, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_CAMERA_HPP_
#define _APP_MODULE_CAMERA_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class camera {
   public:
    camera(MAL* mcu);
    void init();
    void update();
    bool AttackColor;

    struct camera_object {
        uint16_t ball_angle;
        uint16_t ball_distance;
    };

    uint16_t angle[3] = {0};
    uint8_t distance[3] = {0};
    bool enable[3] = {false};

    uint16_t angle;
    uint16_t distance;

   private:
    MAL* _mcu;
    void _read_by_header();
    void _read_via_buffer();

    int _rx_mode = 0;

    union {
        uint8_t camera_rx_buffer[8];
        camera_object data;
    } camera_rx_data;
};

#endif /* _APP_MODULE_CAMERA_HPP_ */