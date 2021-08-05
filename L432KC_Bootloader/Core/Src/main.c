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

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void vL432kc_DeInitAndJump(const uint32_t u32JumpAddress);

// From https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Inc/bootloader.h
typedef void (application_t)(void);

// From https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Inc/bootloader.h
typedef struct
{
    uint32_t    stack_addr;     // Stack Pointer
    application_t*  func_p;        // Program Counter
} JumpStruct;


static void vL432kc_DeInitAndJump(const uint32_t u32FwAddress)
{
  uint32_t u32VectorAddress = 0;

  uint32_t* pu32FwFlashVectorTablePointer = (uint32_t*)u32FwAddress;
  uint32_t* pu32FwRamVectorTablePointer = (uint32_t*)RAM_VECTOR_TABLE_BEGIN;

  // Check if we need to do reset handler relocation. Not 100% accurate because
  // if original binary reset handler gets pushed back beyond "natural" 0x5000 border, this fails
  uint32_t u32UnalteredResetAddress = *(pu32FwFlashVectorTablePointer + 1);

  // Cannot figure out right now what corner case could be
  if (u32UnalteredResetAddress < FLASH_FIRMWARES_EARLIEST_BEGIN)
  {
    // Detected actual firmware, so copy and patch it.

    // Copy first
    while (pu32FwRamVectorTablePointer < (uint32_t*)RAM_VECTOR_TABLE_END)
    {
      *(pu32FwRamVectorTablePointer++) = *(pu32FwFlashVectorTablePointer++);
    }
    pu32FwRamVectorTablePointer = (uint32_t*)RAM_VECTOR_TABLE_BEGIN;
    // Reset is in offset 1
    // Example
    // We are given  u32FwAddress = 0x8005000;
    // Firmware binary thinks it is in 0x8000000 (which is actually bootloader start address)
    // Offset is 0x8005000 - 0x8000000 eq u32FwAddress - FLASH_BOOTLOADER_BEGIN

    // Patch Error_Handler first
    *(pu32FwRamVectorTablePointer + 1) += (u32FwAddress - FLASH_BOOTLOADER_BEGIN);

    /*
    // Optionally, patch the rest of the vector table
    pu32FwRamVectorTablePointer = (uint32_t*)RAM_VECTOR_TABLE_BEGIN;
    pu32FwRamVectorTablePointer++;
    pu32FwRamVectorTablePointer++;

    while (pu32FwRamVectorTablePointer < (uint32_t*)RAM_VECTOR_TABLE_END)
    {
      *(pu32FwRamVectorTablePointer++) += (u32FwAddress - FLASH_BOOTLOADER_BEGIN);
    }
    //*/


    u32VectorAddress = RAM_VECTOR_TABLE_BEGIN;
  }
  else
  {
    u32VectorAddress = u32FwAddress;
  }
  // Deinitialization and jump parts from
  // https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Inc/bootloader.h
  const JumpStruct* pxJumpVector = (JumpStruct*)u32VectorAddress;

  HAL_GPIO_DeInit(LD3_GPIO_Port, LD3_Pin);
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  HAL_RCC_DeInit();
  HAL_DeInit();
  __disable_irq();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  SCB->VTOR = u32VectorAddress;

  // Actual jump
  asm("msr msp, %0; bx %1;" : : "r"(pxJumpVector->stack_addr), "r"(pxJumpVector->func_p));
}


/**
  * @brief  The application entry point.
  * @retval int
  */

//extern char __KERNEL_BEGIN__;


int main(void)
{
  uint32_t u32LedCounter = 0;
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  // Run high frequency for a brief while, then jump
  for (u32LedCounter = 0; u32LedCounter < 0x120000; u32LedCounter++)
  {
    if ((u32LedCounter % 0xFFFF) == 0)
    {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }
  // Leave LED off
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  // Deinit and jump

  //vL432kc_DeInitAndJump(0x8000000); // 0x8000000 here => bootloader jumps to itself :)

  vL432kc_DeInitAndJump(0x8005000); // Here actual firmware address
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
