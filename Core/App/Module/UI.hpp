/*
 *  UI.hpp
 *
 *  Created on: Jan 10, 2024
 * 
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_UI_HPP_
#define _APP_MODULE_UI_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class UI {
    public:
        UI(MAL* mcu);
        void buzzer(uint16_t pulse, uint8_t tim);
        uint8_t getRotarySW();
        bool getSW();
        void setLED(uint8_t pin, bool states);
    private:
        MAL* _mcu;
};

#endif /* _APP_MODULE_UI_HPP_ */