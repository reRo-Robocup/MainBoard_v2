/*
 * UI.hpp
 *
 *  Created on: Jan 10, 2024
 */

#ifndef APP_DEVICES_DRIVER_UI_HPP_
#define APP_DEVICES_DRIVER_UI_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class Buzzer {
    public:
        Buzzer(MAL* mcu);
        void on(uint16_t pulse, uint8_t ms);
    private:
        MAL* _mcu;
};

class RotarySW {
    public:
        RotarySW(MAL* mcu);
        uint8_t getVal();
    private:
        MAL* _mcu;
};

class Switch {
    public:
        Switch(MAL* mcu);
        bool getStates();
    private:
        MAL* _mcu;
};

#endif