L432KC-BL-FW

# NUCLEO L432KC Bootloader and PIC Firmware Image

This repository is for implementing how to get STM32 NUCLEO-L432KC evaluation board to boot position independent code (pic) firmware image. It is actual jump boot, not some blob function call thing.

Quick statuses

Bootloader: **Works. Automaticly jumps to first non-empty position in firmware area.**

Firmware_anywhere: **Works with all addresses sufficiently aligned to 4 bytes when booted by bootloader. Works also standalone.**

One can try for example these firmware addresses when booting via bootloader, they have all been tested as working:

0x8005000

0x8005200

0x8005204

0x8005208

0x8005400
