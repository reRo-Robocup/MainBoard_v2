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
        uint16_t angle;
        uint16_t distance;
        bool enable;
    };

    struct camera_object ball;
    struct camera_object at_goal;
    struct camera_object df_goal;

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