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
        void updateFPS();
        bool AttackColor;

        uint16_t angle[3] = {0};
        uint8_t distance[3] = {0};
        bool enable[3] = {false};

    private:
        MAL* _mcu;
        void _read_by_header();
        void _read_via_buffer();
};

#endif /* _APP_MODULE_CAMERA_HPP_ */