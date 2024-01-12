/*
 *  LineSensor.hpp
 *
 *  Created on: Dec 7, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_LINESENSOR_HPP_
#define _APP_MODULE_LINESENSOR_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class LineSensor {
   public:
    LineSensor(MAL* mcu);
    void init();
    void update();
    void read();
    void setThreshold();
    uint16_t sensorValue[32] = {0};
    bool isSensorONline[32] = {0};
    bool isonLine;
    uint16_t angle;

   private:
    MAL* _mcu;
    uint32_t _threshold[32] = {0};
    float _sincosTable[32][2] = {0.0};
    const bool SigPattern[16][4] = {
        0
    };
};

#endif /* _APP_MODULE_LINESENSOR_HPP_ */