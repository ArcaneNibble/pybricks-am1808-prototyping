#******************************************************************************
#
# makefile - Rules for building the platform library.
#
# Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
#  ALL RIGHTS RESERVED
#
#******************************************************************************

#
# Locating the root directory 
#
ROOT=../../../../../../

#
# Device and EVM definitions
#
DEVICE=am1808
EVM=evmAM1808

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/build/armv5/cgt/makedefs

#
# Source files 
#
SOURCE=${PLATFORM_SRC}/uart.c        \
       ${PLATFORM_SRC}/i2c.c         \
       ${PLATFORM_SRC}/gpio.c        \
       ${PLATFORM_SRC}/emac.c        \
       ${PLATFORM_SRC}/mcasp.c       \
       ${PLATFORM_SRC}/spi.c         \
       ${PLATFORM_SRC}/rtc.c         \
       ${PLATFORM_SRC}/lcd.c         \
       ${PLATFORM_SRC}/emifa.c       \
       ${PLATFORM_SRC}/ehrpwm.c      \
       ${PLATFORM_SRC}/lidd.c        \
       ${PLATFORM_SRC}/syscfg.c      \
       ${PLATFORM_SRC}/uartConsole.c \
       ${PLATFORM_SRC}/edma.c        \
       ${PLATFORM_SRC}/sysdelay.c    \
       ${PLATFORM_SRC}/timertick.c   \
       ${PLATFORM_SRC}/usb.c         \
       ${PLATFORM_SRC}/vpif.c

#
# Rules for building the source files
#
all: debug release

debug: 
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/platform.lib *.o*
	@mv *.o* Debug/
	@mkdir -p ${PLATFORM_BIN}/Debug
	@cp Debug/platform.lib ${PLATFORM_BIN}/Debug/platform.lib

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/platform.lib *.o*
	@mv *.o* Release/
	@mkdir -p ${PLATFORM_BIN}/Release
	@cp Release/platform.lib ${PLATFORM_BIN}/Release/platform.lib

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${PLATFORM_BIN}/Debug ${PLATFORM_BIN}/Release
