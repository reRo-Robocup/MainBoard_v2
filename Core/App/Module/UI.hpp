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
    void init();
    void update();
    void buzzer(uint32_t Hz, uint32_t tim);
    uint8_t getRotarySW();
    bool getSW();
    void setLED(uint8_t pin, bool states);
    void Lchika();

   private:
    MAL* _mcu;
    uint16_t _buzzer_mode = 0;
    uint32_t _buzzer_start_time;
    uint16_t _buzzer_time;
    uint32_t _buzzer_Hz;
};

#endif /* _APP_MODULE_UI_HPP_ */