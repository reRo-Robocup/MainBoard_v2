/*
 *  BallControll.cpp
 *
 *  Created on: Jan 22, 2024
 * 
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_BALLCONTROLL_HPP_
#define _APP_MODULE_BALLCONTROLL_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class BallControll {
    public:
        BallControll(MAL* mcu);
        void init();

    private:
        MAL* _mcu;
};

#endif /* _APP_MODULE_BALLCONTROLL_HPP_ */