/*
 * Functions for scanning the flash for firmware images and running the images.
 *
 * v. 1.0 / 2022-01-15 / Janne Paalijarvi, StackLake Ltd
 *
 * Features and constraints:
 * - Portable between different cores, tested Cortex-M0 and Cortex-M4
 * - Buffered scan: Uses 512 byte read buffer to speed things up
 * - Firmware start and vector table need to be aligned to 512 bytes
 */
#include "flash_scan_and_jump.h"

#include <string.h>

// This needs to be exactly 512 for things to work
#define FLASH_BUFFER_SIZE 512


/*
 * Function for scanning the flash until firmware image is found.
 *
 * The pu32JumpAddress parameter in is the initial address to start scanning
 * the flash. If we find better address, we set it via the parameter.
 * If we find nothing, the address is not changed.
 */
void vScanMainFirmwareFlashAddress(uint32_t* pu32JumpAddress,
                                   uint32_t u32FlashFwAreaBegin,
                                   uint32_t u32FlashFwAreaEndBoundary)
{
  uint32_t u32ReadNum = 0;
  uint32_t* pu32FwFlashReadPointer = (uint32_t*)(*pu32JumpAddress);
  uint32_t u32JumpAddress = *pu32JumpAddress;
  uint8_t au8EmptyFlashBuffer[FLASH_BUFFER_SIZE] = { 0 };
  uint8_t au8ReadFlashBuffer[FLASH_BUFFER_SIZE] = { 0 };
  uint32_t u32MaxBufReads = (u32FlashFwAreaEndBoundary - u32FlashFwAreaBegin) / FLASH_BUFFER_SIZE;
  uint8_t u8Continue = 1;

  memset(au8EmptyFlashBuffer, 0xFF, sizeof(au8EmptyFlashBuffer));

  for (u32ReadNum = 0; (u32ReadNum < u32MaxBufReads) && u8Continue; u32ReadNum++)
  {
    memcpy(au8ReadFlashBuffer, pu32FwFlashReadPointer, FLASH_BUFFER_SIZE);

    if (memcmp(au8ReadFlashBuffer, au8EmptyFlashBuffer, FLASH_BUFFER_SIZE) != 0)
    {
      // Found something
      u32JumpAddress = (uint32_t)pu32FwFlashReadPointer;
      // Need to go trough in 4 byte increments and see what is here
      // Use the same things
      for (u32ReadNum = 0; (u32ReadNum < (FLASH_BUFFER_SIZE / 4)) && u8Continue; u32ReadNum++)
      {
        if (memcmp(au8EmptyFlashBuffer, pu32FwFlashReadPointer + (u32ReadNum * 4), 4) != 0)
        {
          u32JumpAddress += (u32ReadNum * 4);
          u8Continue = 0;
        }
      }
    }
    pu32FwFlashReadPointer += (FLASH_BUFFER_SIZE / 4);
  }
  if (u8Continue == 0)
  {
    *pu32JumpAddress = u32JumpAddress;
  }
}


/*
 * Function for jumping to/executing the main firmware we found.
 *
 * In order to jump, we need some addresses from the firmware image.
 * So the function reads 2 (4 byte) words from the image at given address.
 * These contain reset handler address and stack pointer address. The
 * firmware image in flash has the reset handler address pointing to
 * beginning of flash, we need to add offset to it to get the jump address.
 *
 * Simple checksum of firmware address is created with XOR. This is needed
 * so that a standalone firmware knows if it is running relocated or not.
 * If checksum matches, it is running relocated and takes appropriate
 * action. We need to also be able to see that during the jump from
 * bootloader to firmware image the address data has not been corrupted.
 *
 * If platform-specific system deinitialization function is passed,
 * it is also executed. Without it some peripherals could be in an
 * unexpected state.
 *
 * Finally we store firmware address, firmware offset and the checksum
 * calculated earlier. We store them to registers. This storing should
 * be one of the last steps before jump. Finally we jump to the firmware
 * image by setting the stack pointer and branching to the address we
 * calculated earlier.
 *
 * Example run:
 * u32BootloaderAddress = 0x8000000
 * u32FirmwareAddress   = 0x8005200
 *
 * u32FirmwareOffset = 0x8005200 - 0x8000000 = 0x5200
 *
 * u32FirmwareStackPointerAddress (is read from 0x8005200) = ?
 * u32FirmwareResetHandlerAddress (is read from 0x8005204) = 0x8000000
 *
 * u32FirmwareResetHandlerAddress = u32FirmwareResetHandlerAddress + u32FirmwareOffset
 *                                = 0x8000000 + 0x5200 = 0x8005200
 *
 * u32RegistersChecksum = u32FirmwareAddress ^ u32FirmwareOffset
 *                      = 0x8005200 ^ 0x5200 = 0x8000000
 *
 * Jump to u32FirmwareResetHandlerAddress = 0x8005200
 *
 *
 *
 * (Why re-calculate 0x8005200 you might ask? When we read it from 0x8005204,
 * it might be something different than 0x8000000. Might be 0x8000200. In this
 * case we would need to jump to 0x8000400. The method shown here makes sure
 * that we jump to the right address.)
 */
void vDeInitAndJumpToMainFirmware(tDeinitFuncPointer vPlatformSpecificDeinit,
                                  uint32_t u32BootloaderAddress,
                                  uint32_t u32FirmwareAddress)
{
  uint32_t u32FirmwareStackPointerAddress = 0;
  uint32_t u32FirmwareResetHandlerAddress = 0;

  uint32_t u32FirmwareOffset = u32FirmwareAddress - u32BootloaderAddress;
  uint32_t* pu32FwFlashPointer = (uint32_t*)u32FirmwareAddress;
  uint32_t u32RegistersChecksum = 0;

  // Read 4 first bytes from FW, the stack pointer
  u32FirmwareStackPointerAddress = *pu32FwFlashPointer;
  // Read 4 next bytes from FW, reset handler address
  pu32FwFlashPointer++;
  u32FirmwareResetHandlerAddress = *pu32FwFlashPointer;
  // Patch it with offset
  u32FirmwareResetHandlerAddress += u32FirmwareOffset;

  // Calculate simple checksum of the registers to be passed
  u32RegistersChecksum = u32FirmwareAddress ^ u32FirmwareOffset;

  if (vPlatformSpecificDeinit != NULL)
  {
    vPlatformSpecificDeinit();
  }

  // Everything set; time to store addresses bootloader is going to pass to
  // firmware via registers. After this the firmware knows what should be done and
  // it does things (system memory remapping, vector table things,
  // global offset table operations) autonomously.

  // Store firmware absolute address to r10 (hoop in case we have limited Cortex-M0)
  asm ("ldr r6, %0; mov r10, r6"
      :"=m"(u32FirmwareAddress)
      :
      :);

  // Store firmware offset to r11 (hoop in case we have limited Cortex-M0)
  asm ("ldr r6, %0; mov r11, r6;"
      :"=m"(u32FirmwareOffset)
      :
      :);

  // Store registers checksum to r12 (hoop in case we have limited Cortex-M0)
  asm ("ldr r6, %0; mov r12, r6;"
      :"=m"(u32RegistersChecksum)
      :
      :);

  // Actual jump
  asm("mov sp, %0; bx %1;" : : "r"(u32FirmwareStackPointerAddress), "r"(u32FirmwareResetHandlerAddress));

}
