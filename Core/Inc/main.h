/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

#include "stm32h5xx_nucleo.h"
#include <stdio.h>

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
#define ADC_PT_7_Pin GPIO_PIN_0
#define ADC_PT_7_GPIO_Port GPIOC
#define ADC_PT_6_Pin GPIO_PIN_1
#define ADC_PT_6_GPIO_Port GPIOC
#define ADC_PT_5_Pin GPIO_PIN_2
#define ADC_PT_5_GPIO_Port GPIOC
#define ADC_PT_4_Pin GPIO_PIN_3
#define ADC_PT_4_GPIO_Port GPIOC
#define ADC_PT_3_Pin GPIO_PIN_0
#define ADC_PT_3_GPIO_Port GPIOA
#define ADC_PT_2_Pin GPIO_PIN_1
#define ADC_PT_2_GPIO_Port GPIOA
#define ADC_PT_1_Pin GPIO_PIN_6
#define ADC_PT_1_GPIO_Port GPIOA
#define MTR_R1_Pin GPIO_PIN_7
#define MTR_R1_GPIO_Port GPIOA
#define ADC_PT_0_Pin GPIO_PIN_4
#define ADC_PT_0_GPIO_Port GPIOC
#define MTR_L4_Pin GPIO_PIN_5
#define MTR_L4_GPIO_Port GPIOC
#define MTR_L2_Pin GPIO_PIN_0
#define MTR_L2_GPIO_Port GPIOB
#define MTR_L3_Pin GPIO_PIN_1
#define MTR_L3_GPIO_Port GPIOB
#define MTR_R4_Pin GPIO_PIN_2
#define MTR_R4_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_12
#define IMU_SDA_GPIO_Port GPIOB
#define MTR_L1_Pin GPIO_PIN_13
#define MTR_L1_GPIO_Port GPIOB
#define MTR_R2_Pin GPIO_PIN_14
#define MTR_R2_GPIO_Port GPIOB
#define MTR_R3_Pin GPIO_PIN_15
#define MTR_R3_GPIO_Port GPIOB
#define SWL_Pin GPIO_PIN_6
#define SWL_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_7
#define LCD_DC_GPIO_Port GPIOC
#define SWR_Pin GPIO_PIN_8
#define SWR_GPIO_Port GPIOC
#define IR_7_Pin GPIO_PIN_9
#define IR_7_GPIO_Port GPIOC
#define IR_6_Pin GPIO_PIN_8
#define IR_6_GPIO_Port GPIOA
#define IR_5_Pin GPIO_PIN_9
#define IR_5_GPIO_Port GPIOA
#define IR_4_Pin GPIO_PIN_10
#define IR_4_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define SPI3_SCK_Pin GPIO_PIN_10
#define SPI3_SCK_GPIO_Port GPIOC
#define SPI3_MOSI_Pin GPIO_PIN_12
#define SPI3_MOSI_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LCD_BK_Pin GPIO_PIN_4
#define LCD_BK_GPIO_Port GPIOB
#define IR_3_Pin GPIO_PIN_5
#define IR_3_GPIO_Port GPIOB
#define IR_2_Pin GPIO_PIN_6
#define IR_2_GPIO_Port GPIOB
#define IR_1_Pin GPIO_PIN_7
#define IR_1_GPIO_Port GPIOB
#define IR_0_Pin GPIO_PIN_8
#define IR_0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define KEY_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_13

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
