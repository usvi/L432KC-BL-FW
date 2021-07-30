L432KC-BL-FW


# NUCLEO L432KC Bootloader and PIC Firmware Image research

This repository is for researching how to get STM32 NUCLEO-L432KC evaluation board to boot position independent firmware image code from an *arbitrary* location using custom bootloader. So to boot the same firmware blob from whatever location. (Of course the bootloader must know the address.)

Quick statuses

Bootloader: **Works. Can jump to application firmware and to even itself :D**

Firmware: **Does not work. Temporarily running from first flash address also with -fPIC on, crashes, investigating**


# Research

## 2021-07-30
### 23:33
I have created a bootloader which jumps to 0x8005000. I have made a main firmware and I have configured it to run from 0x8005000. I'm now researching a way to make it relocatable.

Bootloader boots the main firmware from static location just fine.

I'm able to run the main firmware in debugger directly, passing the bootloader. This is great.

But if I now enable Position Indementend Code / -fPIC from project properties => C/C++ Build => Settings => Tool Settings => MCU GCC Compiler => Miscellaneous , the main firmware fails. Good, now I have a point to debug "bit by bit". At least now I know that -fPIC has an effect.

### 23:43
I temporarily put main firmware to 0x8000000, adjusted linker script to the address and disabled vector table offset magic in system_stm32l4xx.c . Not committed, at least not yet, just experimenting.

Now main firmware starts directly from start and of course always when pressing reset. Lets experiment with -fPIC.

Unbelievable. It ends in hardfault!

## 2021-07-31

### 1:34
Committed the PIC version. Yes, it crashes, investigating crash.

### 2:12
![](cm4fw_001_hardfault_cfsr.jpg)

![](cm4fw_002_cfsr_address.jpg)

![](cm4fw_003_cfsr_overview.jpg)

![](cm4fw_004_cfsr_bits.jpg)

![](cm4fw_005_bfar_address.jpg)

![](cm4fw_006_bfar_overview.jpg)

![](cm4fw_007_bfar_fault_address.jpg)

So it was trying to fetch 0xF8DFFBF8? There is nothing there.
