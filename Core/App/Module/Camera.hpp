/*
 *  Camera.cpp
 *
 *  Created on: Jan 1, 2024
 * 
 *  Author: User, G4T1PR0
 */

#ifndef __CAMERA_CPP__
#define __CAMERA_CPP__

#include <baseMcuAbstractionLayer.hpp>

class camera {
    public:
        camera(MAL* mcu);
        void init();
        void updateFPS();
        bool AttackColor;

        struct Object {};

    private:
        MAL* _mcu;
};

#endif