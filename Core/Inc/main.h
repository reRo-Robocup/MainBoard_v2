/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Rotary_IN3_Pin GPIO_PIN_13
#define Rotary_IN3_GPIO_Port GPIOC
#define isMotorEnabled_Pin GPIO_PIN_15
#define isMotorEnabled_GPIO_Port GPIOC
#define IMU_SDI_Pin GPIO_PIN_1
#define IMU_SDI_GPIO_Port GPIOC
#define IMU_SDO_Pin GPIO_PIN_2
#define IMU_SDO_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_3
#define IMU_CS_GPIO_Port GPIOC
#define ADC_LineA_Pin GPIO_PIN_0
#define ADC_LineA_GPIO_Port GPIOA
#define ADC_LineB_Pin GPIO_PIN_1
#define ADC_LineB_GPIO_Port GPIOA
#define PC_TX2_Pin GPIO_PIN_2
#define PC_TX2_GPIO_Port GPIOA
#define PC_RX2_Pin GPIO_PIN_3
#define PC_RX2_GPIO_Port GPIOA
#define ADC_Vbatt_Pin GPIO_PIN_4
#define ADC_Vbatt_GPIO_Port GPIOA
#define ADC_BallCatchA_Pin GPIO_PIN_5
#define ADC_BallCatchA_GPIO_Port GPIOA
#define ADC_BallCatchB_Pin GPIO_PIN_6
#define ADC_BallCatchB_GPIO_Port GPIOA
#define DebugLED2_Pin GPIO_PIN_7
#define DebugLED2_GPIO_Port GPIOA
#define DebugLED0_Pin GPIO_PIN_4
#define DebugLED0_GPIO_Port GPIOC
#define DebugLED1_Pin GPIO_PIN_5
#define DebugLED1_GPIO_Port GPIOC
#define MuxA0_Pin GPIO_PIN_0
#define MuxA0_GPIO_Port GPIOB
#define MuxA1_Pin GPIO_PIN_1
#define MuxA1_GPIO_Port GPIOB
#define MuxA2_Pin GPIO_PIN_2
#define MuxA2_GPIO_Port GPIOB
#define Kicker_LA_Pin GPIO_PIN_10
#define Kicker_LA_GPIO_Port GPIOB
#define Kicker_LB_Pin GPIO_PIN_12
#define Kicker_LB_GPIO_Port GPIOB
#define IMU_SCLK_Pin GPIO_PIN_13
#define IMU_SCLK_GPIO_Port GPIOB
#define DebugSW_Pin GPIO_PIN_15
#define DebugSW_GPIO_Port GPIOB
#define CAM_TX6_Pin GPIO_PIN_6
#define CAM_TX6_GPIO_Port GPIOC
#define CAM_RX6_Pin GPIO_PIN_7
#define CAM_RX6_GPIO_Port GPIOC
#define PWM_Buzzer_Pin GPIO_PIN_8
#define PWM_Buzzer_GPIO_Port GPIOC
#define Rotary_IN0_Pin GPIO_PIN_9
#define Rotary_IN0_GPIO_Port GPIOC
#define PWM_M1_Pin GPIO_PIN_8
#define PWM_M1_GPIO_Port GPIOA
#define PWM_M2_Pin GPIO_PIN_9
#define PWM_M2_GPIO_Port GPIOA
#define PWM_M3_Pin GPIO_PIN_10
#define PWM_M3_GPIO_Port GPIOA
#define PWM_M4_Pin GPIO_PIN_11
#define PWM_M4_GPIO_Port GPIOA
#define ESC_PWM1_Pin GPIO_PIN_15
#define ESC_PWM1_GPIO_Port GPIOA
#define Rotary_IN1_Pin GPIO_PIN_10
#define Rotary_IN1_GPIO_Port GPIOC
#define Rotary_IN2_Pin GPIO_PIN_11
#define Rotary_IN2_GPIO_Port GPIOC
#define Kicker_H_Pin GPIO_PIN_12
#define Kicker_H_GPIO_Port GPIOC
#define MuxA3_Pin GPIO_PIN_4
#define MuxA3_GPIO_Port GPIOB
#define MuxB0_Pin GPIO_PIN_5
#define MuxB0_GPIO_Port GPIOB
#define MuxB1_Pin GPIO_PIN_6
#define MuxB1_GPIO_Port GPIOB
#define MuxB2_Pin GPIO_PIN_7
#define MuxB2_GPIO_Port GPIOB
#define MuxB3_Pin GPIO_PIN_8
#define MuxB3_GPIO_Port GPIOB
#define ESC_PWM2_Pin GPIO_PIN_9
#define ESC_PWM2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
