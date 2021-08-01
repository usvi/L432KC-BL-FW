L432KC-BL-FW


# NUCLEO L432KC Bootloader and PIC Firmware Image research

This repository is for researching how to get STM32 NUCLEO-L432KC evaluation board to boot position independent firmware image code from an *arbitrary* location using custom bootloader. So to boot the same firmware blob from whatever location. (Of course the bootloader must know the address.)

Quick statuses

Bootloader: **Works. Can jump to application firmware and to even itself :D**

Firmware_0x8005000: **Works when .bin flashed to 0x8005000. Bootloader jumps to the firmware fine. Flash defined in linker to start from 0x8005000. -fPIC enabled in IDE, also fixing VTOR.**

Firmware_anywhere: **Does not work when .bin flashed to 0x8005000. Flash defined in linker to start from 0x8000000. -fPIC enabled in IDE. No VTOR fixups.**


# Research

See the subdirectory [research](research/) .
