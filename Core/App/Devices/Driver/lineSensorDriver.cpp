/*
 * lineSensorDriver.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <Devices/Driver/lineSensorDriver.hpp>

lineSensorDriver::lineSensorDriver(MAL* mcu) {
    _mcu = mcu;
}

void lineSensorDriver::update() {
    for (int i = 0; i < 16; i++) {
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig0, MuxA_SigPattern[i][0]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig1, MuxA_SigPattern[i][1]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig2, MuxA_SigPattern[i][2]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxA_Sig3, MuxA_SigPattern[i][3]);

        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig0, MuxB_SigPattern[i][0]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig1, MuxB_SigPattern[i][1]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig2, MuxB_SigPattern[i][2]);
        _mcu->gpioSetValue(MAL::Peripheral_GPIO::MuxB_Sig3, MuxB_SigPattern[i][3]);

        //_mcu->wait_ms();
        while (!_mcu->isAdcConvCplt(MAL::Peripheral_ADC::MuxA))
            ;

        sensorValue[i]      = _mcu->adcGetValue(MAL::Peripheral_ADC::MuxA);
        sensorValue[i + 16] = _mcu->adcGetValue(MAL::Peripheral_ADC::MuxB);
    }
}