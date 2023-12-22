/*
 * lineSensorDriver.hpp
 *
 *  Created on: Dec 7, 2023
 */

#ifndef APP_DEVICES_DRIVER_LINESENSORDRIVER_H_
#define APP_DEVICES_DRIVER_LINESENSORDRIVER_H_
#include <Devices/McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class lineSensorDriver {
   public:
    lineSensorDriver(MAL* mcu);
    void update();
    uint16_t sensorValue[32] = {0};

   private:
    MAL* _mcu;
    /*
    const bool Mux_SigPattern[16][4] = {
        {1, 1, 1, 0},  // 0
        {0, 1, 1, 0},  // 1
        {1, 0, 1, 0},  // 2
        {0, 0, 1, 0},  // 3
        {1, 1, 0, 0},  // 4
        {0, 1, 0, 0},  // 5
        {1, 0, 0, 0},  // 6
        {0, 0, 0, 0},  // 7
        {1, 1, 1, 1},  // 8
        {0, 1, 1, 1},  // 9
        {1, 0, 1, 1},  // 10
        {0, 0, 1, 1},  // 11
        {1, 1, 0, 1},  // 12
        {0, 1, 0, 1},  // 13
        {1, 0, 0, 1},  // 14
        {0, 0, 0, 1}   // 15
    };*/
    const bool MuxA_SigPattern[16][4] = {
        {1, 1, 1, 0},  // In0 S7
        {0, 1, 1, 0},  // In1 S6
        {1, 0, 1, 0},  // In2 S5
        {0, 0, 1, 0},  // In3 S4
        {1, 1, 0, 0},  // In4 S3
        {0, 1, 0, 0},  // In5 S2
        {1, 0, 0, 0},  // In6 S1
        {0, 0, 0, 0},  // In7 S0
        {1, 1, 1, 1},   // In8 S15
        {0, 1, 1, 1},  // In9 S14
        {1, 0, 1, 1},  // In10 S13
        {0, 0, 1, 1},  // In11 S12
        {1, 1, 0, 1},  // In12 S11
        {0, 1, 0, 1},  // In13 S10
        {1, 0, 0, 1},  // In14 S9
        {0, 0, 0, 1},  // In15 S8
    };
    const bool MuxB_SigPattern[16][4] = {
        {1, 1, 1, 0},  // In0 S7
        {0, 1, 1, 0},  // In1 S6
        {1, 0, 1, 0},  // In2 S5
        {0, 0, 1, 0},  // In3 S4
        {1, 1, 0, 0},  // In4 S3
        {0, 1, 0, 0},  // In5 S2
        {1, 0, 0, 0},  // In6 S1
        {0, 0, 0, 0},  // In7 S0
        {1, 1, 1, 1},   // In8 S15
        {0, 1, 1, 1},  // In9 S14
        {1, 0, 1, 1},  // In10 S13
        {0, 0, 1, 1},  // In11 S12
        {1, 1, 0, 1},  // In12 S11
        {0, 1, 0, 1},  // In13 S10
        {1, 0, 0, 1},  // In14 S9
        {0, 0, 0, 1},  // In15 S8
    };
};

#endif /* APP_DEVICES_DRIVER_LINESENSORDRIVER_H_ */
