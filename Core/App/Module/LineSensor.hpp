/*
 * lineSensorDriver.hpp
 *
 *  Created on: Dec 7, 2023
 */

#ifndef APP_DEVICES_DRIVER_LINESENSOR_HPP_
#define APP_DEVICES_DRIVER_LINESENSOR_HPP_
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

#endif /* APP_DEVICES_DRIVER_LINESENSOR_HPP_ */
