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
    DMA_HandleTypeDef* UART_DMA[MAL::Peripheral_UART::End_U];

    SPI_HandleTypeDef* SPI[MAL::Peripheral_SPI::End_S];
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

    PAL.UART[MAL::Peripheral_UART::Cam] = &huart6;
    PAL.UART[MAL::Peripheral_UART::Debug] = &huart2;

    PAL.SPI[MAL::Peripheral_SPI::IMU] = &hspi2;
}

void stm32f446AbstractionLayer::init() {
    _initADC();
    _initPWM();
    _initUART();
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
}

// ADC
uint16_t stm32f446AbstractionLayer::_data[3][3] = {0};
bool stm32f446AbstractionLayer::_adcCplt[3] = {0};

void stm32f446AbstractionLayer::_initADC(void) {
    if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1], (uint32_t*)this->_data[0], sizeof(uint16_t)) * PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]->Init.NbrOfConversion !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2], (uint32_t*)this->_data[1], sizeof(uint16_t)) * PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2]->Init.NbrOfConversion !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3], (uint32_t*)this->_data[2], sizeof(uint16_t)) * PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3]->Init.NbrOfConversion !=
        HAL_OK) {
        Error_Handler();
    }
}

uint16_t stm32f446AbstractionLayer::adcGetValue(Peripheral_ADC p) {
    return _data[PAL.ADC_Connected[p]][PAL.ADC_RANK[p]];
}

bool stm32f446AbstractionLayer::isAdcConvCplt(Peripheral_ADC p) {
    bool ret = _adcCplt[PAL.ADC_Connected[p]];
    _adcCplt[PAL.ADC_Connected[p]] = false;
    return ret;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
    if (AdcHandle->Instance == PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]->Instance) {
        stm32f446AbstractionLayer::_adcCplt[0] = true;
    } else if (AdcHandle->Instance == PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2]->Instance) {
        stm32f446AbstractionLayer::_adcCplt[1] = true;
    } else if (AdcHandle->Instance == PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3]->Instance) {
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

uint32_t stm32f446AbstractionLayer::getPWMConpare(Peripheral_PWM p) {
    return __HAL_TIM_GET_AUTORELOAD(PAL.PWM_TIM[p]);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM6) {

  }
  if(htim->Instance == TIM7) {
    
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