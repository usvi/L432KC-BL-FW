/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

extern uint32_t __RAM_VECTOR_TABLE_BEGIN__;
extern uint32_t __RAM_VECTOR_TABLE_END__;
extern uint32_t __FLASH_BOOTLOADER_BEGIN__;
extern uint32_t __FLASH_FIRMWARES_EARLIEST_BEGIN__;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// Private...?
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

// Helper defines for addresses
#define RAM_VECTOR_TABLE_BEGIN ((uint32_t)(&__RAM_VECTOR_TABLE_BEGIN__)) /* Basically 0x20000000 */
#define RAM_VECTOR_TABLE_END ((uint32_t)(&__RAM_VECTOR_TABLE_END__)) /* Basically 0x20000200 */
#define FLASH_BOOTLOADER_BEGIN ((uint32_t)(&__FLASH_BOOTLOADER_BEGIN__)) /* Basically 0x8000000 */
#define FLASH_FIRMWARES_EARLIEST_BEGIN ((uint32_t)(&__FLASH_FIRMWARES_EARLIEST_BEGIN__)) /* Basically 0x8005000 */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
