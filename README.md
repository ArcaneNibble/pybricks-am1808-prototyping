# AM1808 StarterWare with minor adaptations for LEGO MINDSTORMS EV3

This repository contains the Texas Instruments AM1808 StarterWare v1.00.03.03.

Some changes were made to make it run with u-boot on LEGO MINDSTORMS EV3:
- Use UART1 in some of the examples, which is the EV3 UART debug Sensor Port 1.
- Modified `exceptionhandler.S` and `startup.c` and `init.S` to be modeled
  after EV3 OSEK.
- A makefile has been added to build and deploy an example project.

Additional changes can be found in the git history. These changes could be
reverted if we manage to build and deploy it in the original form or even
without u-boot. 

# How to build and deploy

The makefile makes the following assumptions:

```
PBTOP := $(TOP)/../pybricks-micropython
UBOOT_FILE = $(PBTOP)/bricks/ev3/u-boot.bin
MAKE_BOOTABLE_IMAGE = $(PBTOP)/bricks/ev3/make_bootable_image.py
```

You just need the `u-boot.bin` file and `make_bootable_image.py`, but if you clone `pybricks-micropython` and this repository in the same parent
folder, then all files are in the right place.

If you are able to build the EV3 target from `pybricks-micropython`, then you
have all the prerequisites to build firmware here too:

```
make
```

To put the EV3 into update mode:
- Ensure the EV3 is off.
- Hold right button while turning on the EV3 with the center button.

If the EV3 is already on, you can reboot it by pressing and holding the center
and back button for about five seconds. If you also hold the right button, it
will boot into update mode, as above.

Connect USB and do:

```
make deploy
```

On boot, the serial output should be similar to:

```
EV3 initialization passed!
Booting EV3 EEprom Boot Loader

	EEprom Version:   0.60
	EV3 Flashtype:    N25Q128A13B

EV3 Booting system 

Jumping to entry point at: 0xC1080000


U-Boot 2019.07-00224-gb10c65ef79-dirty (Nov 12 2024 - 15:19:12 +0100)

I2C:   ready
DRAM:  64 MiB
MMC:   da830-mmc: 0
In:    serial@10c000
Out:   serial@10c000
Err:   serial@10c000
Autoboot in 0 seconds - press 'l' to stop...
Card did not respond to voltage select!
SF: Detected n25q128a13 with page size 256 Bytes, erase size 4 KiB, total 16 MiB
device 0 offset 0x50000, size 0x100000
SF: 1048576 bytes @ 0x50000 Read: OK
## Booting kernel from Legacy Image at c0007fc0 ...
   Image Name:   
   Image Type:   ARM Linux Kernel Image (uncompressed)
   Data Size:    1164 Bytes = 1.1 KiB
   Load Address: c0008000
   Entry Point:  c0008000
   XIP Kernel Image ... OK

Starting kernel ...
```

You should also see the following output corresponding to the example built here.

Your input will be echoed back:
```
StarterWare AM1808 UART echo application.
Hello, world!
```

# Using another u-boot version

Alternatively, change `UBOOT_FILE=/path/to-official-firmware.bin`. Then it will
extract u-boot from the original firmware. Can be used if we think that this
makes any difference.

Since the binaries are very small, we could also change this repository to
build firmware without u-boot to rule out any setup changes performed by
u-boot.

# How to build the other examples

To run other examples, change the following entry in the makefile and add
dependencies as needed:

```
TI_AM1808_PRJ_C = $(addprefix ,\
	examples/evmAM1808/uart/uartEcho.c \
    )
```


# Help wanted

If you manage to run either of the following examples, please open an issue
to let us know!

```
examples/evmAM1808/uart_edma/uartEcho.c
examples/evmAM1808/cache_mmu/uartEdma_Cache.c
```
