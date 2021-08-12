L432KC-BL-FW


# NUCLEO L432KC Bootloader and PIC Firmware Image

This repository is for implementing how to get STM32 NUCLEO-L432KC evaluation board to boot position independent firmware image code from an *arbitrary* location using custom bootloader. So to boot the same firmware blob from whatever location. (Of course the bootloader must know the address.) And we are using real jumps, not calling the blob as library.

Quick statuses

Bootloader: **Works. Can jump to application firmware and to even itself :D**

Firmware_0x8005000: **Works when .bin flashed to 0x8005000. Bootloader jumps to the firmware fine. Flash defined in linker to start from 0x8005000. -fPIC enabled in IDE, also fixing VTOR.**

Firmware_anywhere: **Works with all addresses! Needed extra compiler options and also did a __libc_init_array.**


# Research

See the subdirectory [research](research/) .
