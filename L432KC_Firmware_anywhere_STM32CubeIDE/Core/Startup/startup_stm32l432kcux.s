/**
  ******************************************************************************
  * @file      startup_stm32l432xx.s
  * @author    MCD Application Team
  * @brief     STM32L432xx devices vector table for GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address,
  *                - Configure the clock system
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Apache License, Version 2.0,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/Apache-2.0
  *
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler



/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

#include "..\Inc\pic_conf.h"




.equ  BootRAM,        0xF1E0F85F

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section  .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   sp, =_estack    /* Set stack pointer */











  // Store r10 passed by bootloader as gu32FirmwareAbsPosition, need to use hoop if Cortex-M0
  mov r7, r10
  ldr r2, =gu32FirmwareAbsPosition
  str r7, [r2]

  // Store r11 passed by bootloader as gu32FirmwareOffset, need to use hoop if Cortex-M0
  mov r7, r11
  ldr r2, =gu32FirmwareOffset
  str r7, [r2]

  // Store r12 passed by bootloader as gu32FirmwareAbsOffsetChecksum, need to use hoop if Cortex-M0
  mov r7, r12
  ldr r2, =gu32FirmwareAbsOffsetChecksum
  str r7, [r2]

  // Store vector table RAM being address dynamically so systemconfig can map it
  ldr r7, =__ram_vector_table_begin
  ldr r2, =gu32RamVectorTableBegin
  str r7, [r2]


  // Firmware may be booting as standalone. In that case inspect the checksum
  // and if it does not match, we are most likely running from standalone.
  // Funny thing, Cortex-M0 reset values seem to be like 0xffffffff? Well,
  // checksum in anycase takes care of that correct values are loaded.
  ldr r2, =gu32FirmwareAbsPosition // Load variable address
  ldr r2, [r2] // Load variable data
  ldr r3, =gu32FirmwareOffset // Load variable address
  ldr r3, [r3] // Load variable data
  ldr r4, =gu32FirmwareAbsOffsetChecksum // Load variable address
  ldr r4, [r4] // Load variable data
  movs r1, r2// Calculating the checksum into r1
  eors r1, r1, r3 // r2/gu32FirmwareAbsPosition already there, need only r3/gu32FirmwareOffset
  cmp r1, r4 // Actual compare
  beq BootloadedBootContinue // If match, just do nothing
  // Did not match, so we need to store correct values of gu32FirmwareAbsPosition and gu32FirmwareOffset
  ldr r1, =__flash_begin; // Load variable address
  ldr r2, =gu32FirmwareAbsPosition // Load variable address
  str r1, [r2] // Finally store the new value to ram
  movs r1, #0 // Put zero offset
  ldr r2, =gu32FirmwareOffset // Load firmware offset variable address
  str r1, [r2] // Store zero offset
  // Leave the checksum in memory as it was, even if it was wrong

BootloadedBootContinue:



#ifdef GOT_IN_RAM

GlobalOffsetTableCopyPatchInit:
  movs r0, #0 // Loop variable
  movs r1, #0 // Pointer (just introduction)

GlobalOffsetTableCopyPatchLoopCond:
  ldr r2, =__flash_global_offset_table_begin // Need global offset table table beginning for pointer
  ldr r3, =__flash_global_offset_table_end // And need end for checking loop
  ldr r4, =gu32FirmwareOffset // Need also data offset variable address
  ldr r4, [r4] // And the actual offset value
  adds r2, r2, r4 // Patching flash global offset table begin to honour offset
  adds r3, r3, r4 // Patching flash global offset table end to honour offset
  adds r1, r0, r2 // Pointer value is loop variable + offsetted flash global offset table begin
  cmp r1, r3 // Compare pointer against global offset table flash end
  bhs GlobalOffsetTableCopyPatchEnd // If getting past limits, go to end

GlobalOffsetTableCopyPatchLoopBody:
  ldr r2, [r1] // Load the actual data via pointer
  ldr r3, =__flash_begin // Need flash begin boundary for checking
  ldr r4, =__flash_end // Need also flash end boundary for checking
  cmp r2, r3 // Comparing loaded data to flash begin
  blo GlobalOffsetTableStoreData // If less than flash begin, jump to store
  cmp r2, r4 // Comparing loaded data to flash end
  bhs GlobalOffsetTableStoreData // If more than or equal to end, jump to store

GlobalOffsetTablePatchData:
  ldr r3, =gu32FirmwareOffset // Need data offset variable address
  ldr r3, [r3] // And then the actual data
  adds r2, r2, r3 // Patch the data

GlobalOffsetTableStoreData:
  ldr r3, =__ram_global_offset_table_begin // Get global offset table begin in ram for ram data pointer
  adds r3, r3, r0 // Add loop variable
  str r2, [r3] // Store the data

GlobalOffsetTableLoopIncrements:
  adds r0, r0, #4 // Increment loop
  b GlobalOffsetTableCopyPatchLoopCond // Jump to loop condition checking

GlobalOffsetTableCopyPatchEnd:
  ldr r0, =__ram_global_offset_table_begin
  mov r9, r0 // Stupid trick to put global offset table location to r9, for Cortex-M0

#endif /* #ifdef GOT_IN_RAM */



  // Need to copy and patch vector table in assembly so nobody comes to mess around
VectorTableCopyPatchInit:
  movs r0, #0 // Loop variable
  movs r1, #0 // Pointer (just introduction)

VectorTableCopyPatchLoopCond:
  ldr r2, =__flash_vector_table_begin // Need vector table beginning for pointer
  ldr r3, =__flash_vector_table_end // And need end for checking loop
  ldr r4, =gu32FirmwareOffset // Need also data offset variable address
  ldr r4, [r4] // And the actual offset value
  adds r2, r2, r4 // Patching flash vector table begin to honour offset
  adds r3, r3, r4 // Patching flash vector table end to honour offset
  adds r1, r0, r2 // Pointer value is loop variable + offsetted flash vector table begin
  cmp r1, r3 // Compare pointer against vector table flash end
  bhs VectorTableCopyPatchEnd // If getting past limits, go to end

VectorTableCopyPatchLoopBody:
  ldr r2, [r1] // Load the actual data via pointer
  ldr r3, =__flash_begin // Need flash begin boundary for checking
  ldr r4, =__flash_end // Need also flash end boundary for checking
  cmp r2, r3 // Comparing loaded data to flash begin
  blo VectorTableStoreData // If less than flash begin, jump to store
  cmp r2, r4 // Comparing loaded data to flash end
  bhs VectorTableStoreData // If more than or equal to end, jump to store

VectorTablePatchData:
  ldr r3, =gu32FirmwareOffset // Need data offset variable address
  ldr r3, [r3] // And then the actual data
  adds r2, r2, r3 // Patch the data

VectorTableStoreData:
  ldr r3, =__ram_vector_table_begin // Get vector table begin in ram for ram data pointer
  adds r3, r3, r0 // Add loop variable
  str r2, [r3] // Store the data

VectorTableLoopIncrements:
  adds r0, r0, #4 // Increment loop
  b VectorTableCopyPatchLoopCond // Jump to loop condition checking

VectorTableCopyPatchEnd:



/* Copy the data segment initializers from flash to SRAM */
  movs  r1, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r3, =_sidata
  ldr r7, =gu32FirmwareOffset // Load firmware offset variable address
  ldr r7, [r7] // Load the actual firmware offset variable data
  adds r3, r3, r7 // Patch the sidata location with offset
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds  r1, r1, #4

LoopCopyDataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds  r2, r0, r1
  cmp r2, r3
  bcc CopyDataInit
  ldr r2, =_sbss
  b LoopFillZerobss

/* Zero fill the bss segment. */
FillZerobss:
  // Here we need to check that we are not zeroing out addresses or needed symbols

  ldr r6, =gu32FirmwareAbsPosition // Load address of absolute firmware position variable
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =gu32FirmwareOffset // Load address of firmware offset variable
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =gu32FirmwareAbsOffsetChecksum // Load address of firmware position and offset checksum
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =gu32RamVectorTableBegin // Load address of ram vector table begin variable
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  movs  r3, #0 // Load zero for storing
  str  r3, [r2] // If not escaped yet, make the store

FillZerobssSkip:
  adds r2, r2, #4

LoopFillZerobss:
  ldr r3, = _ebss
  cmp r2, r3
  bcc FillZerobss



/* Call the clock system initialization function.*/
    bl  SystemInit



// Make our own __libc_init_array
CallPreinitsInit:
  ldr r7, =gu32FirmwareOffset
  ldr r7, [r7]
  ldr r0, =__preinit_array_start
  adds r0, r7
  ldr r1, =__preinit_array_end
  adds r1, r7

CallPreinitsLoopCond:
  cmp r0, r1
  beq CallPreinitsEnd// If same, it is at end, go away

CallPreinitsLoop:
  ldr r5, =__init_array_start
  ldr r4, =__init_array_end // Yes, order is funny to say the least
  ldr r3, [r0]
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context
  adds r0, r0, #4
  b CallPreinitsLoopCond

CallPreinitsEnd:
  ldr r3, =_init
  adds r3, r7
  ldr r5, =__init_array_start
  adds r5, r7
  ldr r4, =__init_array_end
  adds r4, r7
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context

CallInitsInit:
  ldr r7, =gu32FirmwareOffset
  ldr r7, [r7]

CallInitsLoopCond:
  cmp r5, r4
  beq CallInitsEnd

CallInitsLoop:
  ldr r3, [r5]
  add r3, r3, r7
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context
  adds r5, r5, #4
  b CallInitsLoopCond

CallInitsEnd:



/* Call the application's entry point.*/
  bl  main

LoopForever:
    b LoopForever

.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex-M4.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section  .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler
  .word PVD_PVM_IRQHandler
  .word TAMP_STAMP_IRQHandler
  .word RTC_WKUP_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Channel1_IRQHandler
  .word DMA1_Channel2_IRQHandler
  .word DMA1_Channel3_IRQHandler
  .word DMA1_Channel4_IRQHandler
  .word DMA1_Channel5_IRQHandler
  .word DMA1_Channel6_IRQHandler
  .word DMA1_Channel7_IRQHandler
  .word ADC1_IRQHandler
  .word CAN1_TX_IRQHandler
  .word CAN1_RX0_IRQHandler
  .word CAN1_RX1_IRQHandler
  .word CAN1_SCE_IRQHandler
  .word EXTI9_5_IRQHandler
  .word TIM1_BRK_TIM15_IRQHandler
  .word TIM1_UP_TIM16_IRQHandler
  .word TIM1_TRG_COM_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word 0
  .word 0
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word 0
  .word 0
  .word SPI1_IRQHandler
  .word 0
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word 0
  .word EXTI15_10_IRQHandler
  .word RTC_Alarm_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word SPI3_IRQHandler
  .word 0
  .word 0
  .word TIM6_DAC_IRQHandler
  .word TIM7_IRQHandler
  .word DMA2_Channel1_IRQHandler
  .word DMA2_Channel2_IRQHandler
  .word DMA2_Channel3_IRQHandler
  .word DMA2_Channel4_IRQHandler
  .word DMA2_Channel5_IRQHandler
  .word 0
  .word 0
  .word 0
  .word COMP_IRQHandler
  .word LPTIM1_IRQHandler
  .word LPTIM2_IRQHandler
  .word USB_IRQHandler
  .word DMA2_Channel6_IRQHandler
  .word DMA2_Channel7_IRQHandler
  .word LPUART1_IRQHandler
  .word QUADSPI_IRQHandler
  .word I2C3_EV_IRQHandler
  .word I2C3_ER_IRQHandler
  .word SAI1_IRQHandler
  .word 0
  .word SWPMI1_IRQHandler
  .word TSC_IRQHandler
  .word 0
  .word 0
  .word RNG_IRQHandler
  .word FPU_IRQHandler
  .word CRS_IRQHandler


/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak PVD_PVM_IRQHandler
  .thumb_set PVD_PVM_IRQHandler,Default_Handler

  .weak TAMP_STAMP_IRQHandler
  .thumb_set TAMP_STAMP_IRQHandler,Default_Handler

  .weak RTC_WKUP_IRQHandler
  .thumb_set RTC_WKUP_IRQHandler,Default_Handler

  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

  .weak DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

  .weak DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

  .weak DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

  .weak DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

  .weak DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler,Default_Handler

  .weak ADC1_IRQHandler
  .thumb_set ADC1_IRQHandler,Default_Handler

  .weak CAN1_TX_IRQHandler
  .thumb_set CAN1_TX_IRQHandler,Default_Handler

  .weak CAN1_RX0_IRQHandler
  .thumb_set CAN1_RX0_IRQHandler,Default_Handler

  .weak CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler,Default_Handler

  .weak CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler,Default_Handler

  .weak EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,Default_Handler

  .weak TIM1_BRK_TIM15_IRQHandler
  .thumb_set TIM1_BRK_TIM15_IRQHandler,Default_Handler

  .weak TIM1_UP_TIM16_IRQHandler
  .thumb_set TIM1_UP_TIM16_IRQHandler,Default_Handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,Default_Handler

  .weak RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler,Default_Handler

  .weak SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak TIM6_DAC_IRQHandler
  .thumb_set TIM6_DAC_IRQHandler,Default_Handler

  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak DMA2_Channel1_IRQHandler
  .thumb_set DMA2_Channel1_IRQHandler,Default_Handler

  .weak DMA2_Channel2_IRQHandler
  .thumb_set DMA2_Channel2_IRQHandler,Default_Handler

  .weak DMA2_Channel3_IRQHandler
  .thumb_set DMA2_Channel3_IRQHandler,Default_Handler

  .weak DMA2_Channel4_IRQHandler
  .thumb_set DMA2_Channel4_IRQHandler,Default_Handler

  .weak DMA2_Channel5_IRQHandler
  .thumb_set DMA2_Channel5_IRQHandler,Default_Handler

  .weak COMP_IRQHandler
  .thumb_set COMP_IRQHandler,Default_Handler

  .weak LPTIM1_IRQHandler
  .thumb_set LPTIM1_IRQHandler,Default_Handler

  .weak LPTIM2_IRQHandler
  .thumb_set LPTIM2_IRQHandler,Default_Handler

  .weak USB_IRQHandler
  .thumb_set USB_IRQHandler,Default_Handler

  .weak DMA2_Channel6_IRQHandler
  .thumb_set DMA2_Channel6_IRQHandler,Default_Handler

  .weak DMA2_Channel7_IRQHandler
  .thumb_set DMA2_Channel7_IRQHandler,Default_Handler

  .weak LPUART1_IRQHandler
  .thumb_set LPUART1_IRQHandler,Default_Handler

  .weak QUADSPI_IRQHandler
  .thumb_set QUADSPI_IRQHandler,Default_Handler

  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler,Default_Handler

  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler,Default_Handler

  .weak SAI1_IRQHandler
  .thumb_set SAI1_IRQHandler,Default_Handler

  .weak SWPMI1_IRQHandler
  .thumb_set SWPMI1_IRQHandler,Default_Handler

  .weak TSC_IRQHandler
  .thumb_set TSC_IRQHandler,Default_Handler

  .weak RNG_IRQHandler
  .thumb_set RNG_IRQHandler,Default_Handler

  .weak FPU_IRQHandler
  .thumb_set FPU_IRQHandler,Default_Handler

  .weak CRS_IRQHandler
  .thumb_set CRS_IRQHandler,Default_Handler
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
