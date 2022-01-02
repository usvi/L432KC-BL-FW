L432KC-BL-FW


# NUCLEO L432KC Bootloader and PIC Firmware Image

This repository is for implementing how to get STM32 NUCLEO-L432KC evaluation board to boot position independent firmware image code from (almost) *arbitrary* location using custom bootloader. So to boot the same firmware blob from whatever location. (Of course the bootloader must know the address.) And we are using real jumps, not calling the blob as library.

Quick statuses

Bootloader: **Works. Can jump to application firmware and to even itself :D**

Firmware_anywhere: **Works with all addresses!**

***Small print***
VTOR and thus basically the whole image needs to be aligned to 512 bytes in Cortex-M4.

One can try for example these addresses:

0x8005000, 0x8005200, 0x8005400, 0x8006C00, 0x8009000, 0x800A200, 0x800E600,
  0x8010C00, 0x801AA00, 0x801D400, 0x8025A00, 0x803D000

I just tried for example the 0x803D000 and it works.
