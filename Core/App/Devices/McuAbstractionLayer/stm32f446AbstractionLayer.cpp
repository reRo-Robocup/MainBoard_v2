/*
 * stm32f446AbstractionLayer.cpp
 *
 *  Created on: Dec 7, 2023
 */

#include <Devices/McuAbstractionLayer/stm32f446AbstractionLayer.hpp>
#include <cstring>
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

struct PeripheralAllocation {
    ADC_HandleTypeDef* ADC_Ins[MAL::Peripheral_ADC::End_A];
    uint8_t ADC_RANK[MAL::Peripheral_ADC::End_A];

    TIM_HandleTypeDef* PWM_TIM[MAL::Peripheral_PWM::End_P];
    uint32_t PWM_CH[MAL::Peripheral_PWM::End_P];

    GPIO_TypeDef* GPIO_PORT[MAL::Peripheral_GPIO::End_G];
    uint16_t GPIO_PIN[MAL::Peripheral_GPIO::End_G];

    UART_HandleTypeDef* UART[MAL::Peripheral_UART::End_U];
    DMA_HandleTypeDef* UART_DMA[MAL::Peripheral_UART::End_U];
};

static PeripheralAllocation PAL;

stm32f446AbstractionLayer::stm32f446AbstractionLayer() {
    PAL.ADC_Ins[MAL::Peripheral_ADC::MuxA] = &hadc1;
    PAL.ADC_Ins[MAL::Peripheral_ADC::MuxB] = &hadc3;
    PAL.ADC_Ins[MAL::Peripheral_ADC::BatteryVoltage] = &hadc2;
    PAL.ADC_Ins[MAL::Peripheral_ADC::BallCatchA] = &hadc2;
    PAL.ADC_Ins[MAL::Peripheral_ADC::BallCatchB] = &hadc2;

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

    PAL.UART[MAL::Peripheral_UART::Cam] = &huart6;
    PAL.UART[MAL::Peripheral_UART::Debug] = &huart2;
}

void stm32f446AbstractionLayer::init() {
    _initADC();
}

// ADC
uint16_t stm32f446AbstractionLayer::_data[2] = {0};
bool stm32f446AbstractionLayer::_adcCplt[2] = {0};

void stm32f446AbstractionLayer::_initADC(void) {
}

uint16_t stm32f446AbstractionLayer::adcGetValue(Peripheral_ADC p) {
}

bool stm32f446AbstractionLayer::isAdcConvCplt(Peripheral_ADC p) {
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
}

// PWM
void stm32f446AbstractionLayer::pwmSetDuty(Peripheral_PWM p, float duty) {
}

// GPIO

void stm32f446AbstractionLayer::gpioSetValue(Peripheral_GPIO p, bool value) {
}

bool stm32f446AbstractionLayer::gpioGetValue(Peripheral_GPIO p) {
}

// UART

uint8_t stm32f446AbstractionLayer::_uartRxBuffer[Peripheral_UART::End_U][UART_BUFFER_SIZE] = {0};

void stm32f446AbstractionLayer::_initUART() {
    while (HAL_UART_Receive_DMA(PAL.UART[MAL::Peripheral_UART::Cam], _uartRxBuffer[MAL::Peripheral_UART::Cam], UART_BUFFER_SIZE) != HAL_OK) {
    }

    while (HAL_UART_Receive_DMA(PAL.UART[MAL::Peripheral_UART::Debug], _uartRxBuffer[MAL::Peripheral_UART::Debug], UART_BUFFER_SIZE) != HAL_OK) {
    }
}

uint32_t stm32f446AbstractionLayer::_uartCheckRxBufferDmaWriteAddress(Peripheral_UART p) {
    if (p != Peripheral_UART::End_U) {
        return (UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(PAL.UART_DMA[p])) % UART_BUFFER_SIZE;
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
        if (_uartCheckRxBufferDmaWriteAddress(p) != _uartRxBufferReadAddress[p]) {
            data = _uartRxBuffer[p][_uartRxBufferReadAddress[p]++];
            _uartRxBufferReadAddress[p] %= UART_BUFFER_SIZE;
        }
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
    uint32_t dmaWriteAddress = _uartCheckRxBufferDmaWriteAddress(p);

    if (dmaWriteAddress > _uartRxBufferReadAddress[p]) {
        uint32_t diff_readAddress2dmaWriteAddress = dmaWriteAddress - _uartRxBufferReadAddress[p];

        if (diff_readAddress2dmaWriteAddress > size) {
            memcpy(data, &_uartRxBuffer[p][_uartRxBufferReadAddress[p]], size);
            _uartRxBufferReadAddress[p] += size;
        } else {
            memcpy(data, &_uartRxBuffer[p][_uartRxBufferReadAddress[p]], diff_readAddress2dmaWriteAddress);
            _uartRxBufferReadAddress[p] = dmaWriteAddress;
        }

    } else if (dmaWriteAddress < _uartRxBufferReadAddress[p]) {
        uint32_t diff_readAddress2BufferEndAddress = UART_BUFFER_SIZE - 1 - _uartRxBufferReadAddress[p];
        uint32_t buff_cp_cnt = 0;

        if (diff_readAddress2BufferEndAddress > size) {
            memcpy(data, &_uartRxBuffer[p][_uartRxBufferReadAddress[p]], size);
            _uartRxBufferReadAddress[p] += size;
        } else {
            memcpy(data, &_uartRxBuffer[p][_uartRxBufferReadAddress[p]], diff_readAddress2BufferEndAddress);
            _uartRxBufferReadAddress[p] = 0;
            buff_cp_cnt = diff_readAddress2BufferEndAddress;

            uint32_t diff_readAddress2dmaWriteAddress = dmaWriteAddress - _uartRxBufferReadAddress[p];

            if (diff_readAddress2dmaWriteAddress > 0) {
                if (diff_readAddress2dmaWriteAddress + buff_cp_cnt > size) {
                    memcpy(&data[diff_readAddress2dmaWriteAddress], &_uartRxBuffer[p][_uartRxBufferReadAddress[p]], size - buff_cp_cnt);
                    _uartRxBufferReadAddress[p] += size - buff_cp_cnt;
                } else {
                    memcpy(&data[diff_readAddress2BufferEndAddress], &_uartRxBuffer[p][_uartRxBufferReadAddress[p]], diff_readAddress2dmaWriteAddress);
                    _uartRxBufferReadAddress[p] = dmaWriteAddress;
                }
            }
        }
    }
}

uint32_t stm32f446AbstractionLayer::uartGetRxDataSize(Peripheral_UART p) {
    uint32_t dmaWriteAddress = _uartCheckRxBufferDmaWriteAddress(p);
    uint32_t size = 0;

    if (dmaWriteAddress > _uartRxBufferReadAddress[p]) {
        size = dmaWriteAddress - _uartRxBufferReadAddress[p];
    } else if (dmaWriteAddress < _uartRxBufferReadAddress[p]) {
        size = UART_BUFFER_SIZE - _uartRxBufferReadAddress[p] + dmaWriteAddress;
    }

    return size;
}
