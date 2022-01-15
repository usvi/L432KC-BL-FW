#ifndef __FLASH_SCAN_AND_JUMP_H
#define __FLASH_SCAN_AND_JUMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*tDeinitFuncPointer)(void);

void vScanMainFirmwareFlashAddress(uint32_t* pu32JumpAddress,
                                   uint32_t u32FlashFwAreaBegin,
                                   uint32_t u32FlashFwAreaEndBoundary);

void vDeInitAndJumpToMainFirmware(tDeinitFuncPointer vPlatformSpecificDeinit,
                                  uint32_t u32BootloaderBeginAddress,
                                  uint32_t u32FwAddress);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_SCAN_AND_JUMP_H */
