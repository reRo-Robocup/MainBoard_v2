/*
 *  BatteryVoltageCheckcer.hpp
 *
 *  Created on: Mar 3, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_BATTERYVOLTAGECHECKER_HPP_
#define _APP_MODULE_BATTERYVOLTAGECHECKER_HPP_

#include <Lib/MovingAverageFilter.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class BatteryVoltageChecker {
   public:
    BatteryVoltageChecker(baseMcuAbstractionLayer* mcu);

    void init();
    void update();
    float getVoltage();
    void setWarningVoltage(float voltage);
    void setCriticalVoltage(float voltage);
    void setWarningTime(uint32_t time);
    void setCriticalTime(uint32_t time);
    void setWarningCallback(void (*callback)(bool));
    void setCriticalCallback(void (*callback)(bool));

   private:
    baseMcuAbstractionLayer* _mcu;
    MovingAverageFilter<float, 50> _filter;

    int _mode;

    float _voltage;
    float _warning_voltage = 0;
    float _critical_voltage = 0;
    uint32_t _warning_time = 0;
    uint32_t _critical_time = 0;
    uint32_t _warning_start_time = 0;
    uint32_t _critical_start_time = 0;
    void (*_warning_callback)(bool);
    void (*_critical_callback)(bool);

    const float _raw2voltage = 3.3f / (1 << 12);
    const float _voltage2batt = 12.6 / 3.3;
};

#endif /* _APP_MODULE_BATTERYVOLTAGECHECKER_HPP_ */