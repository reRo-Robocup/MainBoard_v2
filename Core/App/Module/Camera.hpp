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
        uint16_t yellow_angle;
        uint16_t blue_angle;
        uint16_t ball_distance;
        uint16_t yellow_distance;
        uint16_t blue_distance;
        uint8_t enable;
    };

    camera_object data;

   private:
    MAL* _mcu;
    void _read_by_header();
    void _read_via_buffer();

    int _rx_mode = 0;
    unsigned int _rx_data_index;

    union {
        uint8_t rx_buffer[sizeof(camera_object)];
        camera_object parsed_data;
    } camera_rx_data_parser;
};

#endif /* _APP_MODULE_CAMERA_HPP_ */