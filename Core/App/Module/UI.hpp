/*
 * UI.hpp
 *
 *  Created on: Jan 10, 2024
 */

#ifndef APP_DEVICES_DRIVER_UI_HPP_
#define APP_DEVICES_DRIVER_UI_HPP_

#include <baseMcuAbstractionLayer.hpp>

class Buzzer {
    public:
        Buzzer(MAL* mcu);
        void init();
        void on(uint16_t pulse, uint8_t ms);
    private:
        MAL* _mcu;
};

class RotarySW {
    public:
        RotarySW(MAL* mcu);
        void init();
        uint8_t readVal();
    private:
        MAL* _mcu;
};

#endif