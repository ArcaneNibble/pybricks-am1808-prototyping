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
include ${ROOT}/build/armv5/gcc/makedefs

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
	$(AR) $(ARFLAGS) Debug/libplatform.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${PLATFORM_BIN}/Debug
	@cp Debug/libplatform.a ${PLATFORM_BIN}/Debug/libplatform.a

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libplatform.a *.o*
	@mv *.o* Release/
	@mkdir -p ${PLATFORM_BIN}/Release
	@cp Release/libplatform.a ${PLATFORM_BIN}/Release/libplatform.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${PLATFORM_BIN}/Debug ${PLATFORM_BIN}/Release
