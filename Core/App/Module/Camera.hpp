/*
 *  Camera.cpp
 *
 *  Created on: Jan 1, 2024
 * 
 *  Author: User, G4T1PR0
 */

#ifndef __APP_MODULE_CAMERA_HPP__
#define __APP_MODULE_CAMERA_HPP__

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
};

#endif 