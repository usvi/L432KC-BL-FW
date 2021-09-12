#ifndef _IMAGE_INFO_H_
#define _IMAGE_INFO_H_

#include <stdint.h>

// Direct hovering in debugger yields unpredictible results on
// relocated firmware. But if properly referenced, the value is got just
// fine via got table.
extern uint32_t gu32FirmwareAbsPosition;
extern uint32_t gu32FirmwareOffset;
extern uint32_t gu32FirmwareAbsOffsetChecksum;
extern uint32_t gu32RamVectorTableBegin;

#endif // #define _IMAGE_INFO_H_
