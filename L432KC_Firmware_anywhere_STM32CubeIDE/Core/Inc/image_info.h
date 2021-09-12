#ifndef _IMAGE_INFO_H_
#define _IMAGE_INFO_H_

#include <stdint.h>

// Sometimes howering might show false results in debugger, but these still have right values.
extern uint32_t gu32FirmwareAbsPosition;
extern uint32_t gu32FirmwareOffset;
extern uint32_t gu32FirmwareAbsOffsetChecksum;
extern uint32_t gu32RamVectorTableBegin;

#endif // #define _IMAGE_INFO_H_
