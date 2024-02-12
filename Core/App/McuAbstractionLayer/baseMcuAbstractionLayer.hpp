/*
 *  baseMcuAbstractionLayer.hpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MCUABSTRACTIONLAYER_BASEMCUABSTRACTIONLAYER_HPP_
#define _APP_MCUABSTRACTIONLAYER_BASEMCUABSTRACTIONLAYER_HPP_

#include <stdint.h>

#include <GlobalDefines.h>

class baseMcuAbstractionLayer {
   public:
    enum Peripheral_ADC {
        MuxA,
        MuxB,
        BatteryVoltage,
        BallCatchA,
        BallCatchB,
        End_A,
    };

    enum Peripheral_PWM {
        Motor1,
        Motor2,
        Motor3,
        Motor4,
        Buzzer,
        End_P,
    };

    enum Peripheral_GPIO {
        MuxA_Sig0,
        MuxA_Sig1,
        MuxA_Sig2,
        MuxA_Sig3,
        MuxB_Sig0,
        MuxB_Sig1,
        MuxB_Sig2,
        MuxB_Sig3,
        Debug_SW,
        Debug_LED0,
        Debug_LED1,
        Debug_LED2,
        isMotorEnabled,
        Rotary_IN0,
        Rotary_IN1,
        Rotary_IN2,
        Rotary_IN3,
        IMU_CS,
        End_G,
    };

    enum Peripheral_UART {
        Cam,
        Debug,
        End_U,
    };

    enum Peripheral_SPI {
        IMU,
        End_S,
    };

    enum Peripheral_I2C {
        OLED,
        End_I,
    };

    enum Peripheral_Interrupt {
        T1ms,
        End_T
    };

    virtual void init(void) = 0;

    // ADC
    virtual uint16_t adcGetValue(Peripheral_ADC p) = 0;
    virtual bool isAdcConvCplt(Peripheral_ADC p) = 0;
    virtual void adcWaitConvCplt(Peripheral_ADC p) = 0;

    // PWM
    virtual void pwmSetDuty(Peripheral_PWM p, float duty) = 0;
    virtual uint32_t getPWMConpare(Peripheral_PWM p) = 0;

    // GPIO
    virtual void gpioSetValue(Peripheral_GPIO p, bool value) = 0;
    virtual bool gpioGetValue(Peripheral_GPIO p) = 0;

    // UART
    virtual void uartPutChar(Peripheral_UART p, uint8_t data) = 0;
    virtual uint8_t uartGetChar(Peripheral_UART p) = 0;

    virtual void uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) = 0;
    virtual void uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) = 0;
    virtual uint32_t uartGetRxDataSize(Peripheral_UART p) = 0;

    // SPI
    virtual void spiWriteViaBuffer(Peripheral_SPI p, uint8_t* data, uint32_t size) = 0;
    virtual void spiReadViaBuffer(Peripheral_SPI p, uint8_t* data, uint32_t size) = 0;

    // DELAY
    virtual void delay_ms(uint32_t ms) = 0;
    virtual uint32_t millis(void) = 0;

    // INTERRUPT
    virtual void interruptSetCallback(Peripheral_Interrupt p, void (*callback)(void)) = 0;
};

typedef baseMcuAbstractionLayer MAL;

#endif /* _APP_MCUABSTRACTIONLAYER_BASEMCUABSTRACTIONLAYER_HPP_ */