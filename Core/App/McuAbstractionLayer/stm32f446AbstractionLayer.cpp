/*
 *  stm32f446AbstractionLayer.cpp
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#include "stm32f446AbstractionLayer.hpp"
#include <cstring>
#include "adc.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

struct PeripheralAllocation {
    enum STM_ADC {
        ADC_1,
        ADC_2,
        ADC_3,
        ADC_END
    };
    ADC_HandleTypeDef* ADC_Ins[ADC_END];
    STM_ADC ADC_Connected[MAL::Peripheral_ADC::End_A];
    uint8_t ADC_RANK[MAL::Peripheral_ADC::End_A];

    TIM_HandleTypeDef* PWM_TIM[MAL::Peripheral_PWM::End_P];
    uint32_t PWM_CH[MAL::Peripheral_PWM::End_P];

    GPIO_TypeDef* GPIO_PORT[MAL::Peripheral_GPIO::End_G];
    uint16_t GPIO_PIN[MAL::Peripheral_GPIO::End_G];

    UART_HandleTypeDef* UART[MAL::Peripheral_UART::End_U];

    SPI_HandleTypeDef* SPI[MAL::Peripheral_SPI::End_S];

    TIM_HandleTypeDef* TimerInterrupt_TIM[MAL::Peripheral_Interrupt::End_T];
};

static PeripheralAllocation PAL;

stm32f446AbstractionLayer::stm32f446AbstractionLayer() {
    PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1] = &hadc1;
    PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2] = &hadc2;
    PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3] = &hadc3;

    PAL.ADC_Connected[MAL::Peripheral_ADC::MuxA] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_Connected[MAL::Peripheral_ADC::MuxB] = PeripheralAllocation::STM_ADC::ADC_3;
    PAL.ADC_Connected[MAL::Peripheral_ADC::BatteryVoltage] = PeripheralAllocation::STM_ADC::ADC_2;
    PAL.ADC_Connected[MAL::Peripheral_ADC::BallCatchA] = PeripheralAllocation::STM_ADC::ADC_2;
    PAL.ADC_Connected[MAL::Peripheral_ADC::BallCatchB] = PeripheralAllocation::STM_ADC::ADC_2;

    PAL.ADC_RANK[MAL::Peripheral_ADC::MuxA] = 0;
    PAL.ADC_RANK[MAL::Peripheral_ADC::MuxB] = 0;
    PAL.ADC_RANK[MAL::Peripheral_ADC::BatteryVoltage] = 0;
    PAL.ADC_RANK[MAL::Peripheral_ADC::BallCatchA] = 1;
    PAL.ADC_RANK[MAL::Peripheral_ADC::BallCatchB] = 2;

    PAL.PWM_TIM[MAL::Peripheral_PWM::Motor1] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor1] = TIM_CHANNEL_1;

    PAL.PWM_TIM[MAL::Peripheral_PWM::Motor2] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor2] = TIM_CHANNEL_2;

    PAL.PWM_TIM[MAL::Peripheral_PWM::Motor3] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor3] = TIM_CHANNEL_3;

    PAL.PWM_TIM[MAL::Peripheral_PWM::Motor4] = &htim1;
    PAL.PWM_CH[MAL::Peripheral_PWM::Motor4] = TIM_CHANNEL_4;

    PAL.PWM_TIM[MAL::Peripheral_PWM::Buzzer] = &htim8;
    PAL.PWM_CH[MAL::Peripheral_PWM::Buzzer] = TIM_CHANNEL_3;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxA_Sig0] = MuxA0_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxA_Sig0] = MuxA0_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxA_Sig1] = MuxA1_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxA_Sig1] = MuxA1_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxA_Sig2] = MuxA2_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxA_Sig2] = MuxA2_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxA_Sig3] = MuxA3_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxA_Sig3] = MuxA3_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxB_Sig0] = MuxB0_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxB_Sig0] = MuxB0_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxB_Sig1] = MuxB1_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxB_Sig1] = MuxB1_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxB_Sig2] = MuxB2_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxB_Sig2] = MuxB2_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::MuxB_Sig3] = MuxB3_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::MuxB_Sig3] = MuxB3_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Debug_SW] = DebugSW_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Debug_SW] = DebugSW_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Debug_LED0] = DebugLED0_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Debug_LED0] = DebugLED0_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Debug_LED1] = DebugLED1_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Debug_LED1] = DebugLED1_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Debug_LED2] = DebugLED2_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Debug_LED2] = DebugLED2_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::isMotorEnabled] = isMotorEnabled_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::isMotorEnabled] = isMotorEnabled_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Rotary_IN0] = Rotary_IN0_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Rotary_IN0] = Rotary_IN0_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Rotary_IN1] = Rotary_IN1_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Rotary_IN1] = Rotary_IN1_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Rotary_IN2] = Rotary_IN2_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Rotary_IN2] = Rotary_IN2_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::Rotary_IN3] = Rotary_IN3_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::Rotary_IN3] = Rotary_IN3_Pin;

    PAL.GPIO_PORT[MAL::Peripheral_GPIO::IMU_CS] = IMU_CS_GPIO_Port;
    PAL.GPIO_PIN[MAL::Peripheral_GPIO::IMU_CS] = IMU_CS_Pin;

    // UART
    PAL.UART[MAL::Peripheral_UART::Cam] = &huart6;
    PAL.UART[MAL::Peripheral_UART::Debug] = &huart2;

    // SPI
    PAL.SPI[MAL::Peripheral_SPI::IMU] = &hspi2;

    // Timer Interrupt
    PAL.TimerInterrupt_TIM[MAL::Peripheral_Interrupt::T1ms] = &htim2;
}

void stm32f446AbstractionLayer::init() {
    _initADC();
    _initPWM();
    _initUART();
    _initTimerInterrupt();
}

// ADC
uint16_t stm32f446AbstractionLayer::_data[3][3] = {0};
bool stm32f446AbstractionLayer::_adcCplt[3] = {0};

void stm32f446AbstractionLayer::_initADC(void) {
    if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1], (uint32_t*)this->_data[0], PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]->Init.NbrOfConversion) !=
        HAL_OK) {
        Error_Handler();
    }
    //__HAL_DMA_DISABLE_IT(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]->DMA_Handle, DMA_IT_HT);

    if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2], (uint32_t*)this->_data[1], PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2]->Init.NbrOfConversion) !=
        HAL_OK) {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2]->DMA_Handle, DMA_IT_TC | DMA_IT_HT);

    if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3], (uint32_t*)this->_data[2], PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3]->Init.NbrOfConversion) !=
        HAL_OK) {
        Error_Handler();
    }
    //__HAL_DMA_DISABLE_IT(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3]->DMA_Handle, DMA_IT_HT);
}

uint16_t stm32f446AbstractionLayer::adcGetValue(Peripheral_ADC p) {
    return _data[PAL.ADC_Connected[p]][PAL.ADC_RANK[p]];
}

bool stm32f446AbstractionLayer::adcConvCpltGetFlag(Peripheral_ADC p) {
    return _adcCplt[PAL.ADC_Connected[p]];
}

void stm32f446AbstractionLayer::adcConvCpltClearFlag(Peripheral_ADC p) {
    _adcCplt[PAL.ADC_Connected[p]] = false;
}

void stm32f446AbstractionLayer::adcWaitConvCplt(Peripheral_ADC p) {
    _adcCplt[PAL.ADC_Connected[p]] = false;
    while (_adcCplt[PAL.ADC_Connected[p]] == false) {
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
    if (AdcHandle == PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]) {
        stm32f446AbstractionLayer::_adcCplt[0] = true;
    } else if (AdcHandle == PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2]) {
        stm32f446AbstractionLayer::_adcCplt[1] = true;
    } else if (AdcHandle == PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3]) {
        stm32f446AbstractionLayer::_adcCplt[2] = true;
    }
}

// PWM

void stm32f446AbstractionLayer::_initPWM() {
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::Motor1], PAL.PWM_CH[MAL::Peripheral_PWM::Motor1]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::Motor2], PAL.PWM_CH[MAL::Peripheral_PWM::Motor2]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::Motor3], PAL.PWM_CH[MAL::Peripheral_PWM::Motor3]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::Motor4], PAL.PWM_CH[MAL::Peripheral_PWM::Motor4]);
    HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::Peripheral_PWM::Buzzer], PAL.PWM_CH[MAL::Peripheral_PWM::Buzzer]);
}

void stm32f446AbstractionLayer::pwmSetDuty(Peripheral_PWM p, float duty) {
    if (p != Peripheral_PWM::End_P) {
        __HAL_TIM_SET_COMPARE(PAL.PWM_TIM[p], PAL.PWM_CH[p], duty * __HAL_TIM_GET_AUTORELOAD(PAL.PWM_TIM[p]));
    }
}

void stm32f446AbstractionLayer::pwmSetFrequency(Peripheral_PWM p, uint32_t frequency) {
    if (p != Peripheral_PWM::End_P) {
        uint32_t apb1_timer_clocks;
        uint32_t apb2_timer_clocks;
        uint32_t timer_clock = 0;

        RCC_ClkInitTypeDef RCC_ClkInitStruct;
        uint32_t pFLatency;
        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);
        apb1_timer_clocks = HAL_RCC_GetPCLK1Freq();
        apb2_timer_clocks = HAL_RCC_GetPCLK2Freq();
        apb1_timer_clocks *= (RCC_ClkInitStruct.APB1CLKDivider == RCC_HCLK_DIV1) ? 1 : 2;
        apb2_timer_clocks *= (RCC_ClkInitStruct.APB2CLKDivider == RCC_HCLK_DIV1) ? 1 : 2;

        printf("apb1_timer_clocks: %u\n\r", apb1_timer_clocks);
        printf("apb2_timer_clocks: %u\n\r", apb2_timer_clocks);

        if ((uint32_t)PAL.PWM_TIM[p]->Instance >= APB2PERIPH_BASE) {
            timer_clock = apb2_timer_clocks;
        } else if ((uint32_t)PAL.PWM_TIM[p]->Instance >= APB1PERIPH_BASE) {
            timer_clock = apb1_timer_clocks;
        }

        for (uint32_t prescaler = 0; prescaler < 65536; prescaler++) {
            for (uint32_t period = 0; period < 65536; period++) {
                if ((timer_clock / ((prescaler + 1) * (period + 1))) == frequency) {
                    printf("frequency: %u\n\r", (timer_clock / ((prescaler + 1) * (period + 1))));
                    printf("timer_clock: %u\n\r", timer_clock);
                    printf("prescaler: %u\n\r", prescaler + 1);
                    printf("period: %u\n\r", period + 1);
                    __HAL_TIM_SET_PRESCALER(PAL.PWM_TIM[p], prescaler);
                    __HAL_TIM_SET_AUTORELOAD(PAL.PWM_TIM[p], period);
                    if (__HAL_TIM_GET_COUNTER(PAL.PWM_TIM[p]) >= __HAL_TIM_GET_AUTORELOAD(PAL.PWM_TIM[p])) {
                        PAL.PWM_TIM[p]->Instance->EGR |= TIM_EGR_UG;
                    }
                    __HAL_TIM_SET_CLOCKDIVISION(PAL.PWM_TIM[p], TIM_CLOCKDIVISION_DIV1);
                    return;
                }
            }
        }

        // __HAL_TIM_SET_PRESCALER(PAL.PWM_TIM[p], frequency / 1000 - 1);
        // __HAL_TIM_SET_AUTORELOAD(PAL.PWM_TIM[p], frequency);
        // if (__HAL_TIM_GET_COUNTER(htim) >= __HAL_TIM_GET_AUTORELOAD(htim)) {
        //     htim->Instance->EGR |= TIM_EGR_UG;
        // }
        // __HAL_TIM_SET_CLOCKDIVISION(PAL.PWM_TIM[p], TIM_CLOCKDIVISION_DIV1);
    }
}

// GPIO

void stm32f446AbstractionLayer::gpioSetValue(Peripheral_GPIO p, bool value) {
    if (p != Peripheral_GPIO::End_G) {
        if (value) {
            HAL_GPIO_WritePin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p], GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p], GPIO_PIN_RESET);
        }
    }
}

bool stm32f446AbstractionLayer::gpioGetValue(Peripheral_GPIO p) {
    if (p != Peripheral_GPIO::End_G) {
        return HAL_GPIO_ReadPin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p]) == GPIO_PIN_SET;
    }
    return false;
}

// UART

RingBuffer<uint8_t, UART_BUFFER_SIZE> stm32f446AbstractionLayer::_uartRxBuffer[Peripheral_UART::End_U];

void stm32f446AbstractionLayer::_initUART() {
    while (HAL_UART_Receive_DMA(PAL.UART[MAL::Peripheral_UART::Cam], _uartRxBuffer[MAL::Peripheral_UART::Cam].Buffer, UART_BUFFER_SIZE) != HAL_OK) {
    }

    while (HAL_UART_Receive_DMA(PAL.UART[MAL::Peripheral_UART::Debug], _uartRxBuffer[MAL::Peripheral_UART::Debug].Buffer, UART_BUFFER_SIZE) != HAL_OK) {
    }
}

uint32_t stm32f446AbstractionLayer::_uartCheckRxBufferDmaWriteAddress(Peripheral_UART p) {
    if (p != Peripheral_UART::End_U) {
        return (UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(PAL.UART[p]->hdmarx)) % UART_BUFFER_SIZE;
    }
    return 0;
}

void stm32f446AbstractionLayer::uartPutChar(Peripheral_UART p, uint8_t data) {
    if (p != Peripheral_UART::End_U) {
        while (HAL_UART_Transmit_DMA(PAL.UART[p], &data, 1) != HAL_OK) {
        }
    }
}

uint8_t stm32f446AbstractionLayer::uartGetChar(Peripheral_UART p) {
    uint8_t data = 0;
    if (p != Peripheral_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartCheckRxBufferDmaWriteAddress(p));
        data = _uartRxBuffer[p].pop();
    }
    return data;
}

void stm32f446AbstractionLayer::uartWriteViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) {
    if (p != Peripheral_UART::End_U) {
        while (HAL_UART_Transmit_DMA(PAL.UART[p], data, size) != HAL_OK) {
        }
    }
}

void stm32f446AbstractionLayer::uartReadViaBuffer(Peripheral_UART p, uint8_t* data, uint32_t size) {
    if (p != Peripheral_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartCheckRxBufferDmaWriteAddress(p));
        _uartRxBuffer[p].pop(data, size);
    }
}

uint32_t stm32f446AbstractionLayer::uartGetRxDataSize(Peripheral_UART p) {
    uint32_t size = 0;
    if (p != Peripheral_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartCheckRxBufferDmaWriteAddress(p));
        size = _uartRxBuffer[p].size();
    }
    return size;
}

// SPI

void stm32f446AbstractionLayer::spiWriteViaBuffer(Peripheral_SPI p, uint8_t* data, uint32_t size) {
    HAL_SPI_Transmit(PAL.SPI[p], data, size, 100);
}

void stm32f446AbstractionLayer::spiReadViaBuffer(Peripheral_SPI p, uint8_t* data, uint32_t size) {
    HAL_SPI_Receive(PAL.SPI[p], data, size, 100);
}

// Delay

void stm32f446AbstractionLayer::delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

uint32_t stm32f446AbstractionLayer::millis(void) {
    return HAL_GetTick();
}

// Interrupt

void (*stm32f446AbstractionLayer::_timerInterruptCallback[Peripheral_Interrupt::End_T])(void);

void stm32f446AbstractionLayer::_initTimerInterrupt() {
    if (HAL_TIM_Base_Start_IT(PAL.TimerInterrupt_TIM[MAL::Peripheral_Interrupt::T1ms]) == HAL_ERROR) {
        while (1) {
        }
    }
}

void stm32f446AbstractionLayer::interruptSetCallback(Peripheral_Interrupt p, void (*callback)(void)) {
    if (p != Peripheral_Interrupt::End_T) {
        _timerInterruptCallback[p] = callback;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == PAL.TimerInterrupt_TIM[MAL::Peripheral_Interrupt::T1ms]) {
        if (stm32f446AbstractionLayer::_timerInterruptCallback[MAL::Peripheral_Interrupt::T1ms] != NULL) {
            stm32f446AbstractionLayer::_timerInterruptCallback[MAL::Peripheral_Interrupt::T1ms]();
        }
    }
}
