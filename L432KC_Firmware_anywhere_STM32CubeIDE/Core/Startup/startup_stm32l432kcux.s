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

.global	g_pfnVectors
.global	Default_Handler



/* start address for the initialization values of the .data section.
defined in linker script */
.word	_sidata
/* start address for the .data section. defined in linker script */
.word	_sdata
/* end address for the .data section. defined in linker script */
.word	_edata
/* start address for the .bss section. defined in linker script */
.word	_sbss
/* end address for the .bss section. defined in linker script */
.word	_ebss



.equ  BootRAM,        0xF1E0F85F

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
	ldr   sp, =_estack    /* Set stack pointer */

	// Store r12 passed by bootloader as gu32FirmwareOffset
	ldr r8, =gu32FirmwareOffset
	str r12, [r8]
	// Store r11 passed by bootloader as gu32FirmwareAbsPosition
	ldr r8, =gu32FirmwareAbsPosition
	str r11, [r8]
	movs r8, #0
	movs r11, #0

GotPatchLoopInit:
	movs r0, #0 // Loop variable
GotPatchLoopCond:
	ldr r1, = _got_start_ram
	ldr r2, = _got_end_ram
	subs r2, r2, r1 // How many bytes is the lenght
	cmp r0, r2 // Check if loop is at end
	beq GotPatchEnd // Jump to end if compare equal
GotPatchLoopBody:
	movs r1, r0 // Copy original loop counter value to r1
	adds r0, r0, #4 // Increase original loop counter r0
	ldr r2, = _got_start_ram // Load got ram start
	ldr r3, = _ram_start // Load actual ram start
	subs r2, r2, r3 // r2 now has plain got offset from where ever
	ldr r3, = _flash_start // Start to assemble flash position
	adds r3, r3, r12 // Add firmware offset
	adds r3, r3, r2 // Add plain offset
	adds r3, r3, r1 // Add loop offset to reading from flash
	ldr r3, [r3] // Load actual table data from flash
	ldr r4, =_ram_start // Assemble limit to check if over start of ram, in which case don't modify (it is ram or a peripheral)
	cmp r3, r4 // Compare address from got and start of ram
	bhs GotStoreTableAddressToRam // If address higher or same (hs) than start of ram, branch to copy got address as is
	ldr r4, =_flash_end // Assemble limit to check if over end of flash, in which case something is just wrong, so branch to store and hope for the best
	cmp r3, r4 // Compare address from got and end of flash
	bhs GotStoreTableAddressToRam // If address address higher or same (hs) than end of flash, branch to store got table address data and hope for the best
	ldr r4, =_flash_start // Assemble limit to check if under start of flash, in which case something is just wrong, so branch to store and hope for the best
	cmp r3, r4 // Compare address from got and start of flash
	blo GotStoreTableAddressToRam // If address address lower (lo) than start of flash, branch to store got table address data and hope for the best
	adds r3, r3, r12 // Finally a position in flash. Add the offset.
GotStoreTableAddressToRam:
	ldr r4, =_ram_start// Start getting address in ram where to put the table address value
	adds r4, r4, r2 // Add plain offset of got
	adds r4, r4, r1 // Add the original loop counter (is: 0, 4, 8, 12, ...)
	str r3, [r4] // Add the table address to ram
	b GotPatchLoopCond // And go to check the loop
GotPatchEnd:
	ldr r9, =_got_start_ram // Putting the location of got table to agreed register r9
	movs r0, 0 // Cleaning up the rest, just in case
	movs r1, 0
	movs r2, 0
	movs r3, 0
	movs r4, 0
	movs r5, 0
	movs r6, 0
	movs r7, 0
	movs r8, 0
	movs r10, 0
	movs r11, 0
	movs r12, 0

/* Call the clock system initialization function.*/
    bl  SystemInit

/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr r12, =gu32FirmwareOffset
	ldr r12, [r12]
	ldr	r3, =_sidata
	adds r3, r3, r12
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4

LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	adds r2, r2, r12
	b	LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
	movs	r3, #0
	adds r2, r2, #4 // Increment the loop counter already so ww avoid non-ending loops
	ldr r4, =gu32FirmwareOffset // Get firmware offset variable address
	cmp r2, r4 // Compare address to the address we are going to zero
	beq LoopFillZerobss // Jump away if would otherwise zero it
	ldr r4, =gu32FirmwareAbsPosition // Get firmware abs position variable address
	cmp r2, r4 // Compare address to the address we are going to zero
	beq LoopFillZerobss // Jump away if would otherwise zero it
	subs r2, r2, #4 // Remove our own increment which was needed for special cases
	str	r3, [r2]
	adds r2, #4

LoopFillZerobss:
	ldr r12, =gu32FirmwareOffset
	ldr r12, [r12]
	ldr	r3, =_ebss
	cmp	r2, r3
	bcc	FillZerobss

/* Call static constructors */
//    bl __libc_init_array

// Make our own __libc_init_array
CallPreinitsInit:
	ldr r0, =__preinit_array_start
	adds r0, r12
	ldr r1, =__preinit_array_end
	adds r1, r12
CallPreinitsLoopCond:
	cmp r0, r1
	beq CallPreinitsEnd// If same, it is at end, go away
CallPreinitsLoop:
	ldr r5, =__init_array_start
	ldr r4, =__init_array_end // Yes, order is funny to say the least
	ldr r3, [r0]
	blx r3
	adds r0, r0, #4
	b CallPreinitsLoopCond
CallPreinitsEnd:

	ldr r3, =_init
	adds r3, r12
	ldr r5, =__init_array_start
	adds r5, r12
	ldr r4, =__init_array_end
	adds r4, r12
	blx r3

	// r4, r5 untouched or good, hopefully
CallInitsInit:
CallInitsLoopCond:
	cmp r5, r4
	beq CallInitsEnd
CallInitsLoop:
	ldr r3, [r5]
	adds r3, r3, r12
	blx r3
	adds r5, r5, #4
	b CallInitsLoopCond
CallInitsEnd:
	movs r0, #0
	movs r1, #0
	movs r2, #0
	movs r3, #0
	movs r4, #0
	movs r5, #0


/* Call the application's entry point.*/
	bl	main

LoopForever:
    b LoopForever
    
.size	Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex-M4.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
	.size	g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.word	0
	.word	PendSV_Handler
	.word	SysTick_Handler
	.word	WWDG_IRQHandler
	.word	PVD_PVM_IRQHandler
	.word	TAMP_STAMP_IRQHandler
	.word	RTC_WKUP_IRQHandler
	.word	FLASH_IRQHandler
	.word	RCC_IRQHandler
	.word	EXTI0_IRQHandler
	.word	EXTI1_IRQHandler
	.word	EXTI2_IRQHandler
	.word	EXTI3_IRQHandler
	.word	EXTI4_IRQHandler
	.word	DMA1_Channel1_IRQHandler
	.word	DMA1_Channel2_IRQHandler
	.word	DMA1_Channel3_IRQHandler
	.word	DMA1_Channel4_IRQHandler
	.word	DMA1_Channel5_IRQHandler
	.word	DMA1_Channel6_IRQHandler
	.word	DMA1_Channel7_IRQHandler
	.word	ADC1_IRQHandler
	.word	CAN1_TX_IRQHandler
	.word	CAN1_RX0_IRQHandler
	.word	CAN1_RX1_IRQHandler
	.word	CAN1_SCE_IRQHandler
	.word	EXTI9_5_IRQHandler
	.word	TIM1_BRK_TIM15_IRQHandler
	.word	TIM1_UP_TIM16_IRQHandler
	.word	TIM1_TRG_COM_IRQHandler
	.word	TIM1_CC_IRQHandler
	.word	TIM2_IRQHandler
	.word	0
	.word	0
	.word	I2C1_EV_IRQHandler
	.word	I2C1_ER_IRQHandler
	.word	0
	.word	0
	.word	SPI1_IRQHandler
	.word	0
	.word	USART1_IRQHandler
	.word	USART2_IRQHandler
	.word	0
	.word	EXTI15_10_IRQHandler
	.word	RTC_Alarm_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SPI3_IRQHandler
	.word	0
	.word	0
	.word	TIM6_DAC_IRQHandler
	.word	TIM7_IRQHandler
	.word	DMA2_Channel1_IRQHandler
	.word	DMA2_Channel2_IRQHandler
	.word	DMA2_Channel3_IRQHandler
	.word	DMA2_Channel4_IRQHandler
	.word	DMA2_Channel5_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	COMP_IRQHandler
	.word	LPTIM1_IRQHandler
	.word	LPTIM2_IRQHandler
	.word	USB_IRQHandler
	.word	DMA2_Channel6_IRQHandler
	.word	DMA2_Channel7_IRQHandler
	.word	LPUART1_IRQHandler
	.word	QUADSPI_IRQHandler
	.word	I2C3_EV_IRQHandler
	.word	I2C3_ER_IRQHandler
	.word	SAI1_IRQHandler
	.word	0
	.word	SWPMI1_IRQHandler
	.word	TSC_IRQHandler
	.word	0
	.word	0
	.word	RNG_IRQHandler
	.word	FPU_IRQHandler
	.word	CRS_IRQHandler


/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

  .weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

  .weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

  .weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler

	.weak	PVD_PVM_IRQHandler
	.thumb_set PVD_PVM_IRQHandler,Default_Handler

	.weak	TAMP_STAMP_IRQHandler
	.thumb_set TAMP_STAMP_IRQHandler,Default_Handler

	.weak	RTC_WKUP_IRQHandler
	.thumb_set RTC_WKUP_IRQHandler,Default_Handler

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler

	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler

	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler

	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler

	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler

	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler

	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler

	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler

	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler

	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler

	.weak	DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler

	.weak	DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler

	.weak	DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler

	.weak	ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,Default_Handler

	.weak	CAN1_TX_IRQHandler
	.thumb_set CAN1_TX_IRQHandler,Default_Handler

	.weak	CAN1_RX0_IRQHandler
	.thumb_set CAN1_RX0_IRQHandler,Default_Handler

	.weak	CAN1_RX1_IRQHandler
	.thumb_set CAN1_RX1_IRQHandler,Default_Handler

	.weak	CAN1_SCE_IRQHandler
	.thumb_set CAN1_SCE_IRQHandler,Default_Handler

	.weak	EXTI9_5_IRQHandler
	.thumb_set EXTI9_5_IRQHandler,Default_Handler

	.weak	TIM1_BRK_TIM15_IRQHandler
	.thumb_set TIM1_BRK_TIM15_IRQHandler,Default_Handler

	.weak	TIM1_UP_TIM16_IRQHandler
	.thumb_set TIM1_UP_TIM16_IRQHandler,Default_Handler

	.weak	TIM1_TRG_COM_IRQHandler
	.thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler

	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler

	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler

	.weak	EXTI15_10_IRQHandler
	.thumb_set EXTI15_10_IRQHandler,Default_Handler

	.weak	RTC_Alarm_IRQHandler
	.thumb_set RTC_Alarm_IRQHandler,Default_Handler

	.weak	SPI3_IRQHandler
	.thumb_set SPI3_IRQHandler,Default_Handler

	.weak	TIM6_DAC_IRQHandler
	.thumb_set TIM6_DAC_IRQHandler,Default_Handler

	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler

	.weak	DMA2_Channel1_IRQHandler
	.thumb_set DMA2_Channel1_IRQHandler,Default_Handler

	.weak	DMA2_Channel2_IRQHandler
	.thumb_set DMA2_Channel2_IRQHandler,Default_Handler

	.weak	DMA2_Channel3_IRQHandler
	.thumb_set DMA2_Channel3_IRQHandler,Default_Handler

	.weak	DMA2_Channel4_IRQHandler
	.thumb_set DMA2_Channel4_IRQHandler,Default_Handler

	.weak	DMA2_Channel5_IRQHandler
	.thumb_set DMA2_Channel5_IRQHandler,Default_Handler

	.weak	COMP_IRQHandler
	.thumb_set COMP_IRQHandler,Default_Handler
	
	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler
	
	.weak	LPTIM2_IRQHandler
	.thumb_set LPTIM2_IRQHandler,Default_Handler	
	
	.weak	USB_IRQHandler
	.thumb_set USB_IRQHandler,Default_Handler	
	
	.weak	DMA2_Channel6_IRQHandler
	.thumb_set DMA2_Channel6_IRQHandler,Default_Handler	
	
	.weak	DMA2_Channel7_IRQHandler
	.thumb_set DMA2_Channel7_IRQHandler,Default_Handler	
	
	.weak	LPUART1_IRQHandler
	.thumb_set LPUART1_IRQHandler,Default_Handler	
	
	.weak	QUADSPI_IRQHandler
	.thumb_set QUADSPI_IRQHandler,Default_Handler	
	
	.weak	I2C3_EV_IRQHandler
	.thumb_set I2C3_EV_IRQHandler,Default_Handler	
	
	.weak	I2C3_ER_IRQHandler
	.thumb_set I2C3_ER_IRQHandler,Default_Handler	
	
	.weak	SAI1_IRQHandler
	.thumb_set SAI1_IRQHandler,Default_Handler
	
	.weak	SWPMI1_IRQHandler
	.thumb_set SWPMI1_IRQHandler,Default_Handler
	
	.weak	TSC_IRQHandler
	.thumb_set TSC_IRQHandler,Default_Handler
	
	.weak	RNG_IRQHandler
	.thumb_set RNG_IRQHandler,Default_Handler
	
	.weak	FPU_IRQHandler
	.thumb_set FPU_IRQHandler,Default_Handler
	
	.weak	CRS_IRQHandler
	.thumb_set CRS_IRQHandler,Default_Handler
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/