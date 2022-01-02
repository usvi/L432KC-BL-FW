L432KC-BL-FW

# NUCLEO L432KC Bootloader and PIC Firmware Image

This repository is for implementing how to get STM32 NUCLEO-L432KC evaluation board to boot position independent code (pic) firmware image. It is actual jump boot, not some blob function call thing.

Quick statuses

Bootloader: **Works. Automaticly jumps to first non-empty position in firmware area.**

Firmware_anywhere: **Works with all addresses sufficiently aligned when booted by bootloader. Works also standalone.**

VTOR and thus basically the whole image needs to be aligned to 512 bytes in Cortex-M4.

One can try for example these addresses, they should all work:  0x8005000, 0x8005200,
0x8005400, 0x8006C00, 0x8009000, 0x800A200, 0x800E600, 0x8010C00, 0x801AA00, 0x801D400,
0x8025A00, 0x803D000
