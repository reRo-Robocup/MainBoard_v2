/*
 *  stm32f446AbstractionLayer.hpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MCUBASEABSTRACTIONLAYER_STM32F446ABSTRACTIONLAYER_HPP_
#define _APP_MCUBASEABSTRACTIONLAYER_STM32F446ABSTRACTIONLAYER_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

#define UART_BUFFER_SIZE 64

class stm32f446AbstractionLayer : public baseMcuAbstractionLayer {
   public:
    stm32f446AbstractionLayer();

    virtual void init(void);

    // ADC
    virtual uint16_t adcGetValue(Peripheral_ADC p);
    virtual bool isAdcConvCplt(Peripheral_ADC p);
    static bool _adcCplt[3];

    // PWM
    virtual void pwmSetDuty(Peripheral_PWM p, float duty);
    virtual uint32_t getPWMConpare(Peripheral_PWM p);

    // GPIO
    virtual void gpioSetValue(Peripheral_GPIO p, bool value);
    virtual bool gpioGetValue(Peripheral_GPIO p);

    // UART
    virtual void uartPutChar(Peripheral_UART p, uint8_t data);
    virtual uint8_t uartGetChar(Peripheral_UART p);

    virtual void uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size);
    virtual void uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size);
    virtual uint32_t uartGetRxDataSize(Peripheral_UART p);

    // SPI
    virtual void spiWriteViaBuffer(Peripheral_SPI p, uint8_t* data, uint32_t size);
    virtual void spiReadViaBuffer(Peripheral_SPI p, uint8_t* data, uint32_t size);

    // Delay
    virtual void delay_ms(uint32_t ms);
    virtual uint32_t millis(void);

    // Interrupt
    virtual void interruptSetCallback(Peripheral_Interrupt p, void (*callback)(void));
    static void (*_timerInterruptCallback[Peripheral_Interrupt::End_T])(void);

   private:
    // ADC
    void _initADC();
    static uint16_t _data[3][3];

    // Timer PWM
    void _initPWM();

    // UART
    void _initUART();
    uint32_t _uartCheckRxBufferDmaWriteAddress(Peripheral_UART p);
    static uint8_t _uartRxBuffer[Peripheral_UART::End_U][UART_BUFFER_SIZE];
    uint32_t _uartRxBufferReadAddress[Peripheral_UART::End_U] = {0};

    // Interrupt
    void _initTimerInterrupt();
};

#endif /* _APP_MCUBASEABSTRACTIONLAYER_STM32F446ABSTRACTIONLAYER_HPP_ */