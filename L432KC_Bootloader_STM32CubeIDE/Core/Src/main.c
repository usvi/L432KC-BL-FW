/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <string.h>


extern uint32_t __flash_bootloader_begin;
extern uint32_t __flash_fwarea_begin;
extern uint32_t __flash_fwarea_end;

#define FLASH_BOOTLOADER_BEGIN ((uint32_t)(&__flash_bootloader_begin)) /* Basically 0x8000000 */
#define FLASH_FWAREA_BEGIN ((uint32_t)(&__flash_fwarea_begin)) /* Basically 0x8005000 */
#define FLASH_FWAREA_END_BOUNDARY ((uint32_t)(&__flash_fwarea_end)) /* Basically 0x8040000 */


static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void vL432kc_DeInitAndJump(uint32_t u32JumpAddress);


static void vL432kc_DeInitAndJump(uint32_t u32FwAddress)
{
  uint32_t u32FirmwareStackPointerAddress = 0;
  uint32_t u32FirmwareResetHandlerAddress = 0;

  uint32_t u32FirmwareOffset = u32FwAddress - FLASH_BOOTLOADER_BEGIN;
  uint32_t* pu32FwFlashPointer = (uint32_t*)u32FwAddress;
  uint32_t u32RegistersChecksum = 0;

  // Read 4 first bytes from FW, the stack pointer
  u32FirmwareStackPointerAddress = *pu32FwFlashPointer;
  // Read 4 next bytes from FW, reset handler address
  pu32FwFlashPointer++;
  u32FirmwareResetHandlerAddress = *pu32FwFlashPointer;
  // Patch it with offset
  u32FirmwareResetHandlerAddress += u32FirmwareOffset;

  // Deinitialization and jump parts from
  // https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Inc/bootloader.h

  __disable_irq();

  HAL_GPIO_DeInit(LD3_GPIO_Port, LD3_Pin);
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  // Firmware does all the rest needed, system memory remapping, vector table and got operations, etc.

  // Calculate simple checksum of the registers to be passed
  u32RegistersChecksum = u32FwAddress ^ u32FirmwareOffset;

  // Store firmware absolute address to r10 via hoop if we had Cortex-M0
  // NOTE: INSPECT WITH INSTRUCTION STEPPING MODE THAT r6 IS FREE!
  asm ("ldr r6, %0; mov r10, r6"
      :"=m"(u32FwAddress)
      :
      :);

  // Store firmware offset to r11 via hoop if we had Cortex-M0
  // NOTE: INSPECT WITH INSTRUCTION STEPPING MODE THAT r6 IS FREE!
  asm ("ldr r6, %0; mov r11, r6;"
      :"=m"(u32FirmwareOffset)
      :
      :);

  // Store registers checksum to r12 via hoop if we had Cortex-M0
  // NOTE: INSPECT WITH INSTRUCTION STEPPING MODE THAT r6 IS FREE!
  asm ("ldr r6, %0; mov r12, r6;"
      :"=m"(u32RegistersChecksum)
      :
      :);

  // Actual jump
  asm("mov sp, %0; bx %1;" : : "r"(u32FirmwareStackPointerAddress), "r"(u32FirmwareResetHandlerAddress));

}


/**
  * @brief  The application entry point.
  * @retval int
  */

//extern char __KERNEL_BEGIN__;


int main(void)
{
  uint32_t u32LedCounter = 0;
  uint8_t au8EmptyFlashBuffer[512] = { 0 };
  uint8_t au8ReadFlashBuffer[512] = { 0 };
  uint32_t u32MaxBufReads = (FLASH_FWAREA_END_BOUNDARY - FLASH_FWAREA_BEGIN) / 512;
  uint32_t u32ReadNum = 0;
  uint32_t u32JumpAddress = FLASH_FWAREA_BEGIN; // Default jump address
  uint32_t* pu32FwFlashReadPointer = (uint32_t*)FLASH_FWAREA_BEGIN;
  uint8_t u8Continue = 1;

  memset(au8EmptyFlashBuffer, 0xFF, sizeof(au8EmptyFlashBuffer));

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  // the LED on during our flash scavenging
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

  for (u32ReadNum = 0; (u32ReadNum < u32MaxBufReads) && u8Continue; u32ReadNum++)
  {
    memcpy(au8ReadFlashBuffer, pu32FwFlashReadPointer, 512);

    if (memcmp(au8ReadFlashBuffer, au8EmptyFlashBuffer, 512) != 0)
    {
      // Found something
      u32JumpAddress = (uint32_t)pu32FwFlashReadPointer;
      // Need to go trough in 4 byte increments and see what is here
      // Use the same things
      for (u32ReadNum = 0; (u32ReadNum < (512 / 4)) && u8Continue; u32ReadNum++)
      {
        if (memcmp(au8EmptyFlashBuffer, pu32FwFlashReadPointer + (u32ReadNum * 4), 4) != 0)
        {
          u32JumpAddress += (u32ReadNum * 4);
          u8Continue = 0;
        }
      }
    }
    pu32FwFlashReadPointer += (512/4);
  }
  // Run high frequency for a brief while, then jump
  for (u32LedCounter = 0; u32LedCounter < 0x120000; u32LedCounter++)
  {
    if ((u32LedCounter % 0xFFFF) == 0)
    {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }

  // Finally leave the LED off
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  // Deinit and jump
  vL432kc_DeInitAndJump(u32JumpAddress); // << works if Firmware anywhere flashed here

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
