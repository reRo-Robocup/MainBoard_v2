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
#include <Module/UI.hpp>

class LineSensor {
   public:
    LineSensor(MAL* mcu, UI* ui);
    void init();
    void update();
    void read();
    void setThreshold();
    uint8_t getDisFromCenter();

    uint16_t sensorValue[32] = {0};
    bool sensor_isonline[32] = {0};
    bool isonLine;
    int16_t angle;
    int16_t getMoveAngle(int16_t yaw, int16_t toMove);

   private:
    MAL* _mcu;
    UI* _ui;

    uint32_t _threshold[32] = {0};

    double _sin_table[32];
    double _cos_table[32];
    
    uint8_t _isONline_qty;
    uint8_t _module_r;

    const bool SigPattern[16][4] = {
        {1, 1, 1, 0},  // In0 S7
        {0, 1, 1, 0},  // In1 S6
        {1, 0, 1, 0},  // In2 S5
        {0, 0, 1, 0},  // In3 S4
        {1, 1, 0, 0},  // In4 S3
        {0, 1, 0, 0},  // In5 S2
        {1, 0, 0, 0},  // In6 S1
        {0, 0, 0, 0},  // In7 S0
        {1, 1, 1, 1},  // In8 S15
        {0, 1, 1, 1},  // In9 S14
        {1, 0, 1, 1},  // In10 S13
        {0, 0, 1, 1},  // In11 S12
        {1, 1, 0, 1},  // In12 S11
        {0, 1, 0, 1},  // In13 S10
        {1, 0, 0, 1},  // In14 S9
        {0, 0, 0, 1},  // In15 S8
    };
};

#endif /* _APP_MODULE_LINESENSOR_HPP_ */