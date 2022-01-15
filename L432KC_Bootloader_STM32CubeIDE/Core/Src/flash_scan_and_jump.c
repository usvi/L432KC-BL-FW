#include "flash_scan_and_jump.h"

#include <string.h>

// Parameter in is the initial address. If we find better address, we set it.
void vScanMainFirmwareFlashAddress(uint32_t* pu32JumpAddress,
                                   uint32_t u32FlashFwAreaBegin,
                                   uint32_t u32FlashFwAreaEndBoundary)
{
  uint32_t u32ReadNum = 0;
  uint32_t* pu32FwFlashReadPointer = (uint32_t*)(*pu32JumpAddress);
  uint32_t u32JumpAddress = *pu32JumpAddress;
  uint8_t au8EmptyFlashBuffer[512] = { 0 };
  uint8_t au8ReadFlashBuffer[512] = { 0 };
  uint32_t u32MaxBufReads = (u32FlashFwAreaEndBoundary - u32FlashFwAreaBegin) / 512;
  uint8_t u8Continue = 1;

  memset(au8EmptyFlashBuffer, 0xFF, sizeof(au8EmptyFlashBuffer));

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
  if (u8Continue == 0)
  {
    *pu32JumpAddress = u32JumpAddress;
  }
}

void vDeInitAndJumpToMainFirmware(void (*pvPlatformSpecificDeinit)(void),
                                  uint32_t u32BootloaderBeginAddress,
                                  uint32_t u32FwAddress)
{
  uint32_t u32FirmwareStackPointerAddress = 0;
  uint32_t u32FirmwareResetHandlerAddress = 0;

  uint32_t u32FirmwareOffset = u32FwAddress - u32BootloaderBeginAddress;
  uint32_t* pu32FwFlashPointer = (uint32_t*)u32FwAddress;
  uint32_t u32RegistersChecksum = 0;

  // Read 4 first bytes from FW, the stack pointer
  u32FirmwareStackPointerAddress = *pu32FwFlashPointer;
  // Read 4 next bytes from FW, reset handler address
  pu32FwFlashPointer++;
  u32FirmwareResetHandlerAddress = *pu32FwFlashPointer;
  // Patch it with offset
  u32FirmwareResetHandlerAddress += u32FirmwareOffset;

  pvPlatformSpecificDeinit();
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
