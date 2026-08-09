/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN(OSC32_IN)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT(OSC32_OUT)   ------> RCC_OSC32_OUT
     PH0-OSC_IN(PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT(PH1)   ------> RCC_OSC_OUT
     PA13(JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14(JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PA15(JTDI)   ------> DEBUG_JTDI
     PB3(JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MTR_R1_Pin|IR_6_Pin|IR_5_Pin|IR_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MTR_L4_Pin|LCD_DC_Pin|IR_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MTR_L2_Pin|MTR_L3_Pin|MTR_R4_Pin|MTR_L1_Pin
                          |MTR_R2_Pin|MTR_R3_Pin|IR_3_Pin|IR_2_Pin
                          |IR_1_Pin|IR_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MTR_R1_Pin IR_6_Pin IR_5_Pin IR_4_Pin */
  GPIO_InitStruct.Pin = MTR_R1_Pin|IR_6_Pin|IR_5_Pin|IR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_L4_Pin LCD_DC_Pin IR_7_Pin */
  GPIO_InitStruct.Pin = MTR_L4_Pin|LCD_DC_Pin|IR_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_L2_Pin MTR_L3_Pin MTR_R4_Pin MTR_L1_Pin
                           MTR_R2_Pin MTR_R3_Pin IR_3_Pin IR_2_Pin
                           IR_1_Pin IR_0_Pin */
  GPIO_InitStruct.Pin = MTR_L2_Pin|MTR_L3_Pin|MTR_R4_Pin|MTR_L1_Pin
                          |MTR_R2_Pin|MTR_R3_Pin|IR_3_Pin|IR_2_Pin
                          |IR_1_Pin|IR_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SWL_Pin SWR_Pin */
  GPIO_InitStruct.Pin = SWL_Pin|SWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
