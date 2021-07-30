L432KC-BL-FW


# NUCLEO L432KC Bootloader and PIC Firmware Image research

This repository is for researching how to get STM32 NUCLEO-L432KC evaluation board to boot position independent firmware image code from an *arbitrary* location using custom bootloader. So to boot the same firmware blob from whatever location. (Of course the bootloader must know the address.)

Quick statuses

Bootloader: **Not implemented**

Firmware: **Not implemented**


# Research

## 2021-07-30
### 23:33
I have created a bootloader which jumps to 0x8005000. I have made a main firmware and I have configured it to run from 0x8005000. I'm now researching a way to make it relocatable.

Bootloader boots the main firmware from static location just fine.

I'm able to run the main firmware in debugger directly, passing the bootloader. This is great.

But if I now enable Position Indementend Code / -fPIC from project properties => C/C++ Build => Settings => Tool Settings => MCU GCC Compiler => Miscellaneous , the main firmware fails. Good, now I have a point to debug "bit by bit".
