/*
 *  KickerController.hpp
 *
 *  Created on: Mar 13, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_KICKERCONTROLLER_HPP_
#define _APP_MODULE_KICKERCONTROLLER_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class KickerController {
   public:
    KickerController(baseMcuAbstractionLayer* mcu);
    void init();
    void update();
    void setMode(int mode);
    void setKickTime(unsigned int time);

   private:
    baseMcuAbstractionLayer* _mcu;
    int _mode = 0;
    unsigned long long _kick_start_time = 0;
    unsigned int _kick_time = 100;
};

#endif /* _APP_MODULE_KICKERCONTROLLER_HPP_ */
