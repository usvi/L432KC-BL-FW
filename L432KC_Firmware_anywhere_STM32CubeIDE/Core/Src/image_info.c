#include "image_info.h"

// Direct hovering in debugger yields unpredictible results on
// relocated firmware. But if properly referenced, the value is got just
// fine via got table.
uint32_t gu32FirmwareAbsPosition;
uint32_t gu32FirmwareOffset;
uint32_t gu32FirmwareAbsOffsetChecksum;
uint32_t gu32RamVectorTableBegin;
