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

EV3 Firmware Update mode selected
Enable high speed.... 
EV3 Booting system 

Jumping to entry point at: 0xC1080000
```

You should also see the following output corresponding to the example built here.

Your input will be echoed back:
```
?StarterWare AM1808 UART echo application.
Hello, world!
```

# How to build the other examples

To run other examples, change the following entry in the makefile and add
dependencies as needed:

```
TI_AM1808_PRJ_C = $(addprefix ,\
	examples/evmAM1808/uart/uartEcho.c \
    )
```
