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
        void setBallThreshold(uint8_t valA, uint8_t valB);
        bool isCatch(uint8_t pin);
        void dribble(bool cw, uint8_t speed = 100);
        void kick(uint8_t pin);

    private:
        MAL* _mcu;
        uint16_t _threshold[2];
};

#endif /* _APP_MODULE_BALLCONTROLL_HPP_ */